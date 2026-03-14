#ifndef AIRCON_CONTROLLER_MAIN_
#define AIRCON_CONTROLLER_MAIN_


#include "message_structures.h"


void MAIN_test_relays(void);
void MAIN_get_aircon_info(message_aircon_status_t *info);
void MAIN_get_data_info(message_adc_average_t *info);


#endif // AIRCON_CONTROLLER_MAIN_