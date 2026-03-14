#ifndef AIRCON_CONTROLLER_MESSAGE_STRUCTURES_
#define AIRCON_CONTROLLER_MESSAGE_STRUCTURES_


#include <stdint.h>


typedef enum
{
    AIRCON_SATUS = 10,
    GET_VOLTAGE_AVERAGES = 11,
    TEST_RELAYS = 12,
    GET_CONFIGURATION_DATA = 13,
    SET_CONFIGURATION_DATA = 14,
    NUMBER_OF_MSGS = 5,
} msg_ids_e;


typedef struct __attribute__((packed))
{
    int32_t setpoint;
    int32_t temperature;
    int32_t battery_voltage;
} message_adc_average_t;


typedef struct __attribute__((packed))
{
    uint8_t enabled;
    uint8_t running;
    uint8_t state;
    uint16_t run_time;
} message_aircon_status_t;


typedef enum
{
    PERSISTENT_DATA_SAFE_RUN_TIME = 0,
    PERSISTENT_DATA_COOL_DOWN_TIME = 1,
    PERSISTENT_DATA_BATTERY_FILTER_ALPHA = 2,
    PERSISTENT_DATA_DISPLAY_SLEEP_TIMER = 3
} persistent_data_ids_e;


typedef struct __attribute__((packed))
{
    uint8_t id;
    uint8_t data[];
} message_persistent_data_t;


typedef struct __attribute__((packed))
{
    uint8_t id;
    uint8_t data[];
} msg_header_t;

#endif // AIRCON_CONTROLLER_MESSAGE_STRUCTURES_