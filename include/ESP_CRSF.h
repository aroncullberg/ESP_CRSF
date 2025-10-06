#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include <string.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"



#define TAG "CRSF"

/**
 * @brief struct to hold the configuration of the CRSF
 *
 * @param uart_num the uart controller number to use
 * @param tx_pin the tx pin of the esp uart
 * @param rx_pin the rx pin of the esp uart
 *
 */
typedef struct
{
    uint8_t uart_num;
    uint8_t tx_pin;
    uint8_t rx_pin;
} crsf_config_t;

/**
 * @brief structure for handling 16 channels of data, 11 bits each. Which channel is used depends on transmitter setting
 *
 * @return typedef struct
 */
typedef struct __attribute__((packed))
{
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    unsigned ch16 : 11;
} crsf_channels_t;

/**
 * @brief struct for battery data telemetry
 *
 * @param voltage the voltage of the battery in 10*V (1 = 0.1V)
 * @param current the current of the battery in 10*A (1 = 0.1A)
 * @param capacity the capacity of the battery in mah
 * @param remaining the remaining percentage of the battery
 *
 */
typedef struct __attribute__((packed))
{
    unsigned voltage : 16;  // V * 10 big endian
    unsigned current : 16;  // A * 10 big endian
    unsigned capacity : 24; // mah big endian
    unsigned remaining : 8; // %
} crsf_battery_t;

/**
 * @brief struct for GPS data telemetry
 *
 * @param latitude int32 the latitude of the GPS in degree / 10,000,000 big endian
 * @param longitude int32 the longitude of the GPS in degree / 10,000,000 big endian
 * @param groundspeed uint16 the groundspeed of the GPS in km/h / 10 big endian
 * @param heading uint16 the heading of the GPS in degree/100 big endian
 * @param altitude uint16 the altitude of the GPS in meters, +1000m big endian
 * @param satellites uint8 the number of satellites
 *
 */
typedef struct __attribute__((packed))
{
    int32_t latitude;   // degree / 10,000,000 big endian
    int32_t longitude;  // degree / 10,000,000 big endian
    uint16_t groundspeed;  // km/h / 10 big endian
    uint16_t heading;   // GPS heading, degree/100 big endian
    uint16_t altitude;  // meters, +1000m big endian
    uint8_t satellites; // satellites
} crsf_gps_t;

/**
 * @brief struct for airspeed telemetry
 *
 * @param speed airspeed in 0.1 km/h (hectometers/h)
 *
 */
typedef struct __attribute__((packed))
{
    uint16_t speed;             // Airspeed in 0.1 * km/h (hectometers/h) big endian
} crsf_airspeed_t;

/**
 * @brief struct for attitude telemetry
 *
 * @param pitch Pitch angle (LSB = 100 µrad) big endian
 * @param roll Roll angle (LSB = 100 µrad) big endian
 * @param yaw Yaw angle (LSB = 100 µrad) big endian
 *
 */
typedef struct __attribute__((packed))
{
    int16_t pitch;  // Pitch angle (LSB = 100 µrad) big endian
    int16_t roll;   // Roll angle (LSB = 100 µrad) big endian
    int16_t yaw;    // Yaw angle (LSB = 100 µrad) big endian
} crsf_attitude_t;

/**
 * @brief struct for temperature telemetry header
 *
 * Note: This is only the header. Temperature values follow as a variable-length array
 *
 */
typedef struct __attribute__((packed))
{
    uint8_t temp_source_id;  // Source identifier (0=FC/ESCs, 1=Ambient, etc.)
    // int16_t temperature[];    // Variable length array (up to 20 values)
} crsf_temp_header_t;

/**
 * @brief struct for RPM telemetry header
 *
 * Note: This is only the header. RPM values follow as variable-length array of 24-bit integers
 *
 */
typedef struct __attribute__((packed))
{
    uint8_t rpm_source_id;  // Source identifier (0=Motor 1, 1=Motor 2, etc.)
    // int24_t rpm_value[];    // Variable length array (1-19 values, 3 bytes each)
} crsf_rpm_header_t;


typedef enum
{
    CRSF_TYPE_CHANNELS = 0x16,
    CRSF_TYPE_BATTERY = 0x08,
    CRSF_TYPE_GPS = 0x02,
    CRSF_TYPE_ALTITUDE = 0x09,
    CRSF_TYPE_AIRSPEED = 0x0A,
    CRSF_TYPE_RPM = 0x0C,
    CRSF_TYPE_TEMP = 0x0D,
    CRSF_TYPE_ATTITUDE = 0x1E,
    CRSF_TYPE_FLIGHT_MODE = 0x21,
} crsf_type_t;

typedef enum
{
    CRSF_DEST_FC = 0xC8,
    CRSF_DEST_RADIO = 0xEA
} crsf_dest_t;

/**
 * @brief setup CRSF communication
 *
 * @param config pointer to config of CRSF communication
 */
esp_err_t CRSF_init(crsf_config_t *config);

/**
 * @brief cleanup CRSF communication (stop task and delete UART driver)
 *
 */
esp_err_t CRSF_cleanup(void);

/**
 * @brief copy latest 16 channel data received to the pointer
 *
 * @param channels pointer to receiver buffer
 * @return esp_err_t ESP_OK if successful, ESP_FAIL if no data available
 */
esp_err_t CRSF_receive_channels(crsf_channels_t *channels);

/**
 * @brief send battery data telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param payload pointer to the battery data
 */
void CRSF_send_battery_data(crsf_dest_t dest, crsf_battery_t* payload);

/**
 * @brief send gps data telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param payload pointer to the gps data
 */
void CRSF_send_gps_data(crsf_dest_t dest, crsf_gps_t* payload);

/**
 * @brief send airspeed data telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param payload pointer to the airspeed data
 */
void CRSF_send_airspeed_data(crsf_dest_t dest, crsf_airspeed_t* payload);

/**
 * @brief send flight mode telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param mode_string null-terminated string containing the flight mode (max 32 chars)
 */
void CRSF_send_flight_mode_data(crsf_dest_t dest, const char* mode_string);

/**
 * @brief send attitude telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param payload pointer to the attitude data
 */
void CRSF_send_attitude_data(crsf_dest_t dest, crsf_attitude_t* payload);

/**
 * @brief send temperature telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param source_id identifies the temperature source (0=FC/ESCs, 1=Ambient, etc.)
 * @param temps pointer to array of temperature values in deci-degrees Celsius (e.g., 250 = 25.0°C)
 * @param count number of temperature values (max 20)
 */
void CRSF_send_temp_data(crsf_dest_t dest, uint8_t source_id, const int16_t* temps, uint8_t count);

/**
 * @brief send RPM telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param source_id identifies the RPM source (0=Motor 1, 1=Motor 2, etc.)
 * @param rpms pointer to array of RPM values (signed, negative for reverse rotation)
 * @param count number of RPM values (1-19 max)
 */
void CRSF_send_rpm_data(crsf_dest_t dest, uint8_t source_id, const int32_t* rpms, uint8_t count);

