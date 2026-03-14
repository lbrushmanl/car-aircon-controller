#ifndef AIRCON_CONTROLLER_MESSAGE_HANDLER_
#define AIRCON_CONTROLLER_MESSAGE_HANDLER_


#include "stdint.h"


int MSG_HANDLER_process(const void *data, uint8_t size);
void MSG_HANDLER_serial_interface(void);


#endif // AIRCON_CONTROLLER_MESSAGE_HANDLER_