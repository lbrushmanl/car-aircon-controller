#include "adc_driver.h"
#include "filters.h"
#include "max7219.h"
#include "spi.h"

#include <util/delay.h>


#define NUMBER_OF_SETPOINT_SAMPLES (10U)


static int32_t voltage_samples[NUMBER_OF_SETPOINT_SAMPLES] = {0};
static int32_t current_samples[NUMBER_OF_SETPOINT_SAMPLES] = {0};
static moving_average_filter_t voltage_filter = {0};
static moving_average_filter_t current_filter = {0};


int main(void)
{
    MOVING_AVERAGE_FILTER_init(&voltage_filter, voltage_samples, NUMBER_OF_SETPOINT_SAMPLES);
    MOVING_AVERAGE_FILTER_init(&current_filter, current_samples, NUMBER_OF_SETPOINT_SAMPLES);

    SPI_init(false, false, false, ONETWENTYEIGHT);
    MAX7219_init(8);
    ADC_init(false);

    MAX7219_clear_display();

    while (true)
    {
        _delay_ms(1000);

        int32_t voltage = (((int32_t) ADC_get_sample(0) * 5000) / 1023);
        MOVING_AVERAGE_FILTER_update(&voltage_filter, voltage);

        int32_t current = (((int32_t) ADC_get_sample(1) * 5000) / 1023);
        MOVING_AVERAGE_FILTER_update(&current_filter, current);

        MAX7219_write_number(voltage_filter.result, 0, 4);
        MAX7219_write_number(current_filter.result, 4, 8);
    }

    return 0;
}