#include "basic-uart.h"
#include "filters.h"
#include "pid.h"
#include "pwm.h"
#include "utils.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include <util/delay.h>

// #include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


#define DAC_PORT_1                    (PORTD)
#define DAC_PORT_2                    (PORTB)
#define DAC_DDR_1                     (DDRD)
#define DAC_DDR_2                     (DDRB)
#define DAC_PIN                       _BV(PB4)
#define COMPARATOR_VALUE              (ACSR & _BV(ACO))
#define NUMBER_OF_TEMPERATURE_SAMPLES (10U)
#define TEMPERARUE_Q8                 (8U)
#define FULL_SCALE_TEMPERARUE_Q8      (150 << TEMPERARUE_Q8)


static volatile uint8_t dac_output_compare = 128;
static volatile int32_t measured_temperature = 0;
static int32_t temperature_samples[NUMBER_OF_TEMPERATURE_SAMPLES] = {0};
static moving_average_filter_t temperature_filter = {0};
static pid_controller_t temperature_contoller = {0};


// static inline void dac_update(void)
// {
//     static uint8_t dac_pwm_ticks = 0;
//     SET_PIN(DAC_PORT, DAC_PIN, ++dac_pwm_ticks < dac_output_compare);
// }


// ISR (TIMER0_OVF0_vect)
// {
//     dac_update();
// }


static inline void dac_update(uint8_t setpoint)
{
    DAC_PORT_1 = setpoint >> 1;
    SET_PIN(DAC_PORT_2, DAC_PIN, setpoint & 0x01);
}


static void adc_init(void)
{
    // Configure PWM DAC
    CONFIG_PIN(DAC_DDR_1, 0xFF);
    CONFIG_PIN(DAC_DDR_2, DAC_PIN);
    TCCR0 = _BV(CS00);
    TIMSK = _BV(TOIE0); // Enable timer interupt
    sei();
}


static uint8_t adc_get_sample(void)
{
    int16_t setpoint = 128;
    int16_t prev_setpoint = 0;
    uint8_t half_setpoint = 255;

    while (half_setpoint > 2)
    {
        dac_output_compare = setpoint;
        half_setpoint = (uint8_t) ((abs(setpoint - prev_setpoint) * 50) / 100);
        prev_setpoint = setpoint;
        _delay_us(50);
        setpoint += COMPARATOR_VALUE ? half_setpoint : -half_setpoint;
    }

    return setpoint;
}


int main(void)
{
    PWM_init();
    // UART_init(NULL, 51);
    MOVING_AVERAGE_FILTER_init(&temperature_filter, temperature_samples,
                               NUMBER_OF_TEMPERATURE_SAMPLES);
    PID_init(&temperature_contoller, 41, 10, 0);
    adc_init();

    while (true)
    {
        uint8_t raw = adc_get_sample();
        int32_t sample_q8 = ((int32_t) raw * FULL_SCALE_TEMPERARUE_Q8) / 255;
        measured_temperature = MOVING_AVERAGE_FILTER_update(&temperature_filter, sample_q8);
        // uint8_t duty = PID_update(&temperature_contoller, 25 << 8, sample_q8);
        // PWM_update_duty(duty);
    }

    return 0;
}
