#include "message_handler.h"

#include "basic-uart.h"
#include "main.h"
#include "message_structures.h"
#include "persistent_data.h"
#include "utils.h"

#include <avr/eeprom.h>

#include <stdbool.h>
#include <stdint.h>


static void send_data(msg_ids_e id, const void *data, uint8_t size)
{
    UART_send(size + 1);
    UART_send(id);

    for (uint8_t i = 0; i < size; i++)
    {
        UART_send(*(uint8_t *) (data + i));
    }
}


int MSG_HANDLER_process(const void *data, uint8_t size)
{
    ARG_UNUSED(size);

    msg_header_t *msg_header = (msg_header_t *) (data);
    switch (msg_header->id)
    {
    case AIRCON_SATUS:
    {
        message_aircon_status_t response;
        MAIN_get_aircon_info(&response);
        send_data(msg_header->id, &response, sizeof(response));
        break;
    }

    case GET_VOLTAGE_AVERAGES:
    {
        message_adc_average_t response;
        MAIN_get_data_info(&response);
        send_data(msg_header->id, &response, sizeof(response));
        break;
    }

    case TEST_RELAYS:
    {
        MAIN_test_relays();
        break;
    }

    case GET_CONFIGURATION_DATA:
    {
        message_persistent_data_t *request_persistent_data =
            (message_persistent_data_t *) msg_header->data;


        uint8_t buffer[8] = {0};
        message_persistent_data_t *persistent_data = (message_persistent_data_t *) buffer;
        persistent_data->id = request_persistent_data->id;
        eeprom_read_block(persistent_data->data,
                          persistent_data_access[persistent_data->id].address,
                          persistent_data_access[persistent_data->id].size);

        send_data(msg_header->id, persistent_data,
                  sizeof(message_persistent_data_t) +
                      persistent_data_access[persistent_data->id].size);
        break;
    }

    case SET_CONFIGURATION_DATA:
    {
        message_persistent_data_t *persistent_data = (message_persistent_data_t *) msg_header->data;
        eeprom_write_block(persistent_data->data,
                           persistent_data_access[persistent_data->id].address,
                           persistent_data_access[persistent_data->id].size);

        send_data(msg_header->id, persistent_data,
                  sizeof(message_persistent_data_t) +
                      persistent_data_access[persistent_data->id].size);
        break;
    }

    default:
        return -1;
    }

    return 0;
}


void MSG_HANDLER_serial_interface(void)
{
    static uint16_t timeout = 0;
    if ((UART_get_rx_byte_num() < sizeof(uint8_t)) && (timeout == 0))
    {
        return;
    }

    uint8_t size = *UART_rx_data_start();
    timeout++;

    if ((UART_get_rx_byte_num() - 1) < size)
    {
        if (timeout > 100)
        {
            UART_update_tail(UART_get_rx_byte_num());
            timeout = 0;
        }

        return;
    }

    timeout = 0;
    UART_update_tail(sizeof(size)); // Jump to data

    uint8_t buff[32];
    UART_read_buffer_into_var(buff, size);
    MSG_HANDLER_process(buff, size);
}