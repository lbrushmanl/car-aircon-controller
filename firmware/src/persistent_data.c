#include "persistent_data.h"

#include "message_structures.h"

#include <avr/eeprom.h>

#include <stdbool.h>
#include <stdint.h>


/* EEPROM data */
static configuration_data_t EEMEM configuration_data_addresses = {.magic_number = 0xBD,
                                                                  .safe_continuous_run_time =
                                                                      (60 * 30),
                                                                  .cooldown_time = (60),
                                                                  .battery_filter_alpha = 150,
                                                                  .display_sleep_timer = 15};

persistent_data_info_t persistent_data_access[] = {
    [PERSISTENT_DATA_SAFE_RUN_TIME] = {.address =
                                           &configuration_data_addresses.safe_continuous_run_time,
                                       sizeof(
                                           configuration_data_addresses.safe_continuous_run_time)},
    [PERSISTENT_DATA_COOL_DOWN_TIME] = {.address = &configuration_data_addresses.cooldown_time,
                                        sizeof(configuration_data_addresses.cooldown_time)},
    [PERSISTENT_DATA_BATTERY_FILTER_ALPHA] =
        {.address = &configuration_data_addresses.battery_filter_alpha,
         sizeof(configuration_data_addresses.battery_filter_alpha)},
    [PERSISTENT_DATA_DISPLAY_SLEEP_TIMER] = {
        .address = &configuration_data_addresses.display_sleep_timer,
        sizeof(configuration_data_addresses.display_sleep_timer)}};


bool PERSISTENT_DATA_load(configuration_data_t *configuration_data)
{
    eeprom_read_block(configuration_data, &configuration_data_addresses,
                      sizeof(configuration_data_t));

    if (configuration_data->magic_number != 0xBD)
    {
        configuration_data->cooldown_time = 60;
        configuration_data->safe_continuous_run_time = 60 * 30;
        configuration_data->display_sleep_timer = 15;
        return false;
    }

    return true;
}