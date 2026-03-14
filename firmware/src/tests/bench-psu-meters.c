#include "adc_driver.h"
#include "basic-uart.h"
#include "debouncer.h"
#include "debug.h"
#include "filters.h"
#include "hysteresis.h"
#include "non-preemptive-scheduler.h"
#include "utils.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>


#define NUMBER_OF_SETPOINT_SAMPLES    (10U)
#define NUMBER_OF_TEMPERATURE_SAMPLES (10U)
#define TEMPERARUE_Q8                 (8U)
#define FULL_SCALE_TEMPERARUE_Q8      (40 << TEMPERARUE_Q8)

// Relay Mapping
// PB0 PB1 PB2 PB7 PD5 PD6 PD7
// 3   2   1   7   6   5   4
#define RELAY_1 _BV(PB2)
#define RELAY_2 _BV(PB1)
#define RELAY_3 _BV(PB0)
#define RELAY_4 _BV(PD7)
#define RELAY_5 _BV(PD6)
#define RELAY_6 _BV(PD5)
#define RELAY_7 _BV(PB7)

// Button Mapping
// PD3 PD4
// UP  DOWN
#define UP_BUTTON   _BV(PD3)
#define DOWN_BUTTON _BV(PD4)

// LED Mapping
// PD2
// Status
#define STATUS_LED _BV(PD2)

// ADC Mapping
// Extra Batt Voltage Temperature Setpoint
// A6    A7           A0          A1
#define EXTRA_ADC       (6U)
#define BATTERY_ADC     (7U)
#define TEMPERATURE_ADC (0U)
#define SETPOINT_ADC    (1U)


typedef struct __attribute__((packed))
{
    int32_t setpoint;
    int32_t temperature;
} adc_average_t;

// typedef struct {
//     uint8_t index;
//     uint8_t data[];
// } save_parameters_to_eprom_t;


typedef struct
{
    uint8_t id;
    uint8_t data[];
} msg_header_t;

typedef struct
{
    msg_ids_e id;
    uint8_t request_size;
    uint8_t responce_size;
} payload_info_t;


static payload_info_t payload_size_info[] = {{GET_VOLTAGE_AVERAGES, 0, sizeof(adc_average_t)}};

static debouncer_t button_1_debouncer = {.detected_state = true, .change_ticks = 10};
static debouncer_t button_2_debouncer = {.detected_state = true, .change_ticks = 10};
static int32_t setpoint_samples[NUMBER_OF_SETPOINT_SAMPLES] = {0};
static int32_t temperature_samples[NUMBER_OF_TEMPERATURE_SAMPLES] = {0};
static moving_average_filter_t setpoint_filter = {0};
static moving_average_filter_t temperature_filter = {0};
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] = {0};


// static void temperature_controller_timer_init(void)
// {
//     TCCR1A = 0;
//     TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12);
//     OCR1A = 1562;
//     OCR1B = 1562; // 1s
//     TIMSK1 = _BV(OCIE1B); // Enable timer interupt
// }


static inline void relay_gpio_init(void)
{
    CONFIG_PIN(DDRB, RELAY_1 | RELAY_2 | RELAY_3 | RELAY_7);
    CONFIG_PIN(DDRD, RELAY_4 | RELAY_5 | RELAY_6);
}


static inline void button_gpio_init(void)
{
    CONFIG_PIN(DDRD, UP_BUTTON | DOWN_BUTTON);
}


static inline void gpio_init(void)
{
    relay_gpio_init();
    button_gpio_init();
    // LED init
    CONFIG_PIN(DDRD, STATUS_LED);
}


// static void send_data(msg_ids_e id, const void *data, uint8_t size)
// {
//     UART_send(id);

//     for (uint8_t i = 0; i < size; i++)
//     {
//         UART_send(*(uint8_t *) (data + i));
//     }
// }

static void task_status_led(void)
{
    TOGGLE_PIN(PORTD, STATUS_LED);
}


static void task_update_display(void)
{
    printf("SET %u °C\n", setpoint_filter.result);
    printf("%u °C\n", temperature_filter.result);
}


static void task_get_adc_readings(void)
{
    // ADC_get_sample(0);
    // int32_t sample = (int32_t) get_adc_sample();
    // int32_t sample_q8 = (sample * FULL_SCALE_TEMPERARUE_Q8) / 1023;
    MOVING_AVERAGE_FILTER_update(&temperature_filter, sample_q8);
    MOVING_AVERAGE_FILTER_update(&setpoint_filter, sample_q8);
}


static void task_update_temperature_control(void)
{
    hysteresis_t temperature_hysteresis = {.upper_bound = setpoint_filter.result,
                                           .lower_bound = setpoint_filter.result - (2 << 8)};

    bool enable_aircon = hysteresis_control(&temperature_hysteresis, temperature_filter.result);
    SET_PIN(PORTB, RELAY_1, enable_aircon);
}


int main(void)
{
    UART_init(uart_rx_buffer, 51);
    MOVING_AVERAGE_FILTER_init(&setpoint_filter, setpoint_samples, NUMBER_OF_TEMPERATURE_SAMPLES);
    MOVING_AVERAGE_FILTER_init(&temperature_filter, temperature_samples,
                               NUMBER_OF_TEMPERATURE_SAMPLES);

    gpio_init();

    // adc_channel_config_t adc_sequencer[] = {{.channel = TEMPERATURE_ADC, .result_ptr = &},
    //                                         {.channel = SETPOINT_ADC, .result_ptr = &}};
    // ADC_sequencer_init();
    // ADC_init(true);

    sei(); // Enable interupts

    task_t tasks[] = {
        {.task_fn = task_status_led, .period = 500, .elapsed_time = 20},
        {.task_fn = task_update_display, .period = 1000, .elapsed_time = 63},
        {.task_fn = task_get_adc_readings, .period = 100, .elapsed_time = 0},
        {.task_fn = task_update_temperature_control, .period = 1000, .elapsed_time = 121}};

    while (true)
    {
        TASK_SCHEDULER_update(tasks, sizeof(tasks));
        _delay_ms(1);

        //     if (UART_get_rx_byte_num() < sizeof(msg_header_t))
        //     {
        //         continue;
        //     }

        //     msg_header_t *msg_header = (msg_header_t *) UART_rx_data_start();

        //     switch (msg_header->id)
        //     {
        //         // case VARIABLE_UPDATE:
        //         //     if (UART_get_rx_byte_num() < (sizeof(msg_header_t) +
        //         //     sizeof(pid_gains_t)))
        //         //     {
        //         //         continue;
        //         //     }

        //         //     pid_gains_t gains = {0};
        //         //     UART_read_buffer_into_var(&gains, sizeof(pid_gains_t));
        //         //     temperature_contoller.kp_q12 = gains.kp_q12;
        //         //     temperature_contoller.ki_q12 = gains.ki_q12;
        //         //     temperature_contoller.kd_q12 = gains.kd_q12;
        //         //     UART_update_tail(sizeof(msg_header_t) + sizeof(pid_gains_t));

        //         //     printf("%ld %ld %ld GAINS\n", gains.kp_q12, gains.ki_q12,
        //         //     gains.kd_q12); break;

        //     case GET_VOLTAGE_AVERAGES:
        //     {
        //         // adc_average_t m = {.setpoint = setpoint_filter.result,
        //         //                    .temperature = temperature_filter.result};

        //         // send_data(msg_header->id, &m, sizeof(adc_average_t));
        //         printf("%lu.%lu C\n", setpoint_filter.result >> 8,
        //                ((setpoint_filter.result * 100) >> 8) - ((setpoint_filter.result >> 8) *
        //                100));
        //         printf("%lu.%lu C\n", temperature_filter.result >> 8,
        //                ((temperature_filter.result * 100) >> 8) -
        //                    ((temperature_filter.result >> 8) * 100));
        //         UART_update_tail(sizeof(msg_header_t));
        //         break;
        //     }

        //     default:
        //         UART_update_tail(sizeof(msg_header_t));
        //         break;
        //     }
    }

    return 0;
}
