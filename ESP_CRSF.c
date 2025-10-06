#include <stdio.h>
#include "ESP_CRSF.h"
#include "byteswap.h"

#define RX_BUF_SIZE 1024    //UART buffer size

// CRC8 lookup table (poly 0xd5)
static uint8_t crc8_table[256] = {0};

void generate_CRC(uint8_t poly)
{
    for (int idx=0; idx<256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift=0; shift<8; ++shift)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        crc8_table[idx] = crc & 0xff;
    }
}

// Function to calculate CRC8 checksum
uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--)
    {
        crc = crc8_table[crc ^ *data++];
    }

    return crc;
}


SemaphoreHandle_t xMutex;
static TaskHandle_t rx_task_handle = NULL;

static int uart_num = 1;
static QueueHandle_t uart_queue;
crsf_channels_t received_channels = {0};
crsf_battery_t received_battery = {0};
static bool crsf_initialized = false;


static void rx_task(void *arg)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            if (event.type == UART_DATA ) {
                // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);

                //extract length and type
                uint8_t type = dtmp[2];
                uint8_t length = dtmp[1];
                uint8_t dest = dtmp[0];

                //read the rest of the frame
                uint8_t payload_length = length - 2;
                uint8_t payload[payload_length];

                for (int i = 0; i < payload_length; i++) {
                    payload[i] = dtmp[i+3];
                }

                if (type == CRSF_TYPE_CHANNELS) {

                    xSemaphoreTake(xMutex, portMAX_DELAY);
                    received_channels = *(crsf_channels_t*)payload;
                    // ESP_LOGI(TAG, "%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d", received_channels.ch1, received_channels.ch2, received_channels.ch3, received_channels.ch4, received_channels.ch5, received_channels.ch6, received_channels.ch7, received_channels.ch8, received_channels.ch9, received_channels.ch10, received_channels.ch11, received_channels.ch12, received_channels.ch13, received_channels.ch14, received_channels.ch15, received_channels.ch16);
                    xSemaphoreGive(xMutex);
                }
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

esp_err_t CRSF_init(crsf_config_t *config)
{
    if (crsf_initialized) {
        ESP_LOGW(TAG, "CRSF already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    generate_CRC(0xd5);

    uart_num = config->uart_num;

    //begin uart communication with RX
    uart_config_t uart_config = {
        .baud_rate = 420000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, RX_BUF_SIZE, 10, &uart_queue, 0));

    //create semaphore
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        uart_driver_delete(uart_num);
        return ESP_ERR_NO_MEM;
    }

    //create task
    BaseType_t task_ret = xTaskCreate(rx_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES-1, &rx_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        vSemaphoreDelete(xMutex);
        uart_driver_delete(uart_num);
        return ESP_ERR_NO_MEM;
    }

    crsf_initialized = true;
    ESP_LOGI(TAG, "CRSF initialized successfully");
    return ESP_OK;
}

//receive uart data frame
esp_err_t CRSF_receive_channels(crsf_channels_t *channels)
{
    if (!crsf_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *channels = received_channels;
    xSemaphoreGive(xMutex);
    return ESP_OK;
}

esp_err_t CRSF_cleanup(void)
{
    if (!crsf_initialized) {
        ESP_LOGW(TAG, "CRSF not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Cleaning up CRSF...");

    if (rx_task_handle != NULL) {
        vTaskDelete(rx_task_handle);
        rx_task_handle = NULL;
    }

    if (xMutex != NULL) {
        vSemaphoreDelete(xMutex);
        xMutex = NULL;
    }

    ESP_ERROR_CHECK(uart_driver_delete(uart_num));

    uart_queue = NULL;
    memset(&received_channels, 0, sizeof(received_channels));
    memset(&received_battery, 0, sizeof(received_battery));
    crsf_initialized = false;

    ESP_LOGI(TAG, "CRSF cleanup complete");
    return ESP_OK;
}

/**
 * @brief function sends payload to a destination using uart
 *
 * @param payload pointer to payload of given crsf_type_t
 * @param destination destination for payload, typically CRSF_DEST_FC
 * @param type type of data contained in payload
 * @param payload_length length of the payload type
 */
void CRSF_send_payload(const void* payload, crsf_dest_t destination, crsf_type_t type, uint8_t payload_length)
{
    uint8_t packet[payload_length+4]; //payload + dest + len + type + crc

    packet[0] = destination;
    packet[1] = payload_length+2; // size of payload + type + crc
    packet[2] = type;

    memcpy(&packet[3], payload, payload_length);

    //calculate crc
    unsigned char checksum = crc8(&packet[2], payload_length+1);

    packet[payload_length+3] = checksum;

    //send frame
    uart_write_bytes(uart_num, &packet, payload_length+4);
}

void CRSF_send_battery_data(crsf_dest_t dest, crsf_battery_t* payload)
{
    crsf_battery_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_battery_t*)payload;
    payload_proc->voltage = __bswap16(payload_proc->voltage);
    payload_proc->current = __bswap16(payload_proc->current);
    payload_proc->capacity = __bswap16(payload_proc->capacity) << 8;

    CRSF_send_payload(payload_proc, dest, CRSF_TYPE_BATTERY, sizeof(crsf_battery_t));
}

void CRSF_send_gps_data(crsf_dest_t dest, crsf_gps_t* payload)
{
    crsf_gps_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_gps_t*)payload;
    payload_proc->latitude = __bswap32(payload_proc->latitude);
    payload_proc->longitude = __bswap32(payload_proc->longitude);
    payload_proc->groundspeed = __bswap16(payload_proc->groundspeed);
    payload_proc->heading = __bswap16(payload_proc->heading);
    payload_proc->altitude = __bswap16(payload_proc->altitude);

    CRSF_send_payload(payload_proc, dest, CRSF_TYPE_GPS, sizeof(crsf_gps_t));
}

void CRSF_send_airspeed_data(crsf_dest_t dest, crsf_airspeed_t* payload)
{
    crsf_airspeed_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_airspeed_t*)payload;
    payload_proc->speed = __bswap16(payload_proc->speed);

    CRSF_send_payload(payload_proc, dest, CRSF_TYPE_AIRSPEED, sizeof(crsf_airspeed_t));
}

void CRSF_send_flight_mode_data(crsf_dest_t dest, const char* mode_string)
{
    // Flight mode is a null-terminated string, no byte swapping needed, see crsf telem spec.
    // Calculate length including null terminator
    uint8_t len = 0;
    while (mode_string[len] != '\0' && len < 32) {
        len++;
    }
    len++; // Include null terminator

    CRSF_send_payload(mode_string, dest, CRSF_TYPE_FLIGHT_MODE, len);
}

void CRSF_send_attitude_data(crsf_dest_t dest, crsf_attitude_t* payload)
{
    crsf_attitude_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_attitude_t*)payload;
    payload_proc->pitch = __bswap16(payload_proc->pitch);
    payload_proc->roll = __bswap16(payload_proc->roll);
    payload_proc->yaw = __bswap16(payload_proc->yaw);

    CRSF_send_payload(payload_proc, dest, CRSF_TYPE_ATTITUDE, sizeof(crsf_attitude_t));
}

void CRSF_send_temp_data(crsf_dest_t dest, uint8_t source_id, const int16_t* temps, uint8_t count)
{
    // Limit to max 20 temperatures
    if (count > 20) {
        count = 20;
    }

    // Build payload: source_id + temperature array
    uint8_t payload_size = 1 + (count * 2); // 1 byte for source_id + 2 bytes per temperature
    uint8_t payload[payload_size];

    payload[0] = source_id;

    // Copy temperatures with byte swapping
    for (uint8_t i = 0; i < count; i++) {
        int16_t temp_swapped = __bswap16(temps[i]);
        memcpy(&payload[1 + (i * 2)], &temp_swapped, 2);
    }

    CRSF_send_payload(payload, dest, CRSF_TYPE_TEMP, payload_size);
}

/**
 * @brief Pack a signed 32-bit integer into 24-bit big-endian format
 *
 * @param dest destination buffer (must have 3 bytes available)
 * @param value signed 32-bit value to pack
 */
static void pack_int24_be(uint8_t* dest, int32_t value)
{
    // Pack as big-endian 24-bit signed integer
    // MSB first, LSB last
    dest[0] = (value >> 16) & 0xFF;
    dest[1] = (value >> 8) & 0xFF;
    dest[2] = value & 0xFF;
}

void CRSF_send_rpm_data(crsf_dest_t dest, uint8_t source_id, const int32_t* rpms, uint8_t count)
{
    // Limit to max 19 RPM values
    if (count > 19) {
        count = 19;
    }

    // Build payload: source_id + RPM array (3 bytes per RPM)
    uint8_t payload_size = 1 + (count * 3); // 1 byte for source_id + 3 bytes per RPM
    uint8_t payload[payload_size];

    payload[0] = source_id;

    // Pack each RPM as 24-bit big-endian
    for (uint8_t i = 0; i < count; i++) {
        pack_int24_be(&payload[1 + (i * 3)], rpms[i]);
    }

    CRSF_send_payload(payload, dest, CRSF_TYPE_RPM, payload_size);
}