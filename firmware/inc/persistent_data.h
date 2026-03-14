#ifndef AIRCON_CONTROLLER_PERSISTENT_DATA_
#define AIRCON_CONTROLLER_PERSISTENT_DATA_

#include <stdbool.h>
#include <stdint.h>


typedef struct __attribute__((packed))
{
    uint8_t magic_number;
    uint16_t safe_continuous_run_time;
    uint16_t cooldown_time;
    int32_t battery_filter_alpha;
    int8_t display_sleep_timer;
} configuration_data_t;

typedef struct
{
    void *const address;
    uint8_t size;
} persistent_data_info_t;


extern persistent_data_info_t persistent_data_access[];


bool PERSISTENT_DATA_load(configuration_data_t *configuration_data);


#endif // AIRCON_CONTROLLER_PERSISTENT_DATA_