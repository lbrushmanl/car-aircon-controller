#include "main.h"

#include "adc_driver.h"
#include "basic-uart.h"
#include "debouncer.h"
#include "filters.h"
#include "font-15x18.h"
#include "font-6x7.h"
#include "hysteresis.h"
#include "message_handler.h"
#include "message_structures.h"
#include "non-preemptive-scheduler.h"
#include "number-to-string.h"
#include "persistent_data.h"
#include "ssd1306-driver.h"
#include "twi.h"
#include "utils.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>


#define NUMBER_OF_SETPOINT_SAMPLES    (5U)
#define NUMBER_OF_TEMPERATURE_SAMPLES (5U)
#define FULL_SCALE_TEMPERATURE_Q8     (40 << 8U)
#define FULL_SCALE_VOLTAGE_Q8         (30 << 8U)
#define TO_Q8_TEMPERATURE(sample)     (((int32_t) sample * FULL_SCALE_TEMPERATURE_Q8) >> 10)
#define TO_Q8_VOLTAGE(sample)         (((int32_t) sample * FULL_SCALE_VOLTAGE_Q8) >> 10)

// Relay Mapping
// PB0 PB1 PB2 PB7 PD5 PD6 PD7
// 4   6   7   1   2   5   3
#define RELAY_1 _BV(PB7)
#define RELAY_2 _BV(PD5)
#define RELAY_3 _BV(PD7)
#define RELAY_4 _BV(PB0)
#define RELAY_5 _BV(PD6)
#define RELAY_6 _BV(PB1)
#define RELAY_7 _BV(PB2)

// Button Mapping
// PD3             PD4
// AIRCON_ENABLED  NEXT_SCREEN
#define AIRCON_ENABLED_BUTTON _BV(PD3)
#define NEXT_SCREEN_BUTTON    _BV(PD4)

// LED Mapping
// PD2
// Status
#define STATUS_LED _BV(PD2)


// ADC Mapping
// Extra Batt Voltage Temperature Setpoint
// A6    A7           A0          A1
typedef enum
{
    TEMPERATURE_ADC = 0,
    SETPOINT_ADC = 1,
    EXTRA_ADC = 6,
    BATTERY_ADC = 7,
} adc_channels_e;


typedef enum
{
    AIRCON_OFF = 0,
    AIRCON_ON = 1,
    AIRCON_COOLDOWN = 2,
} aircon_running_states_e;


typedef struct
{
    aircon_running_states_e state;
    uint16_t run_time;
    bool run;
    bool enabled;
} aircon_status_t;


typedef struct
{
    uint16_t setpoint;
    uint16_t temperature;
    uint16_t extra_adc;
    uint16_t battery_voltage;
} raw_adc_samples_t;


typedef enum
{
    DISPLAY_INFO_TEMPERATURE = 0,
    DISPLAY_INFO_BATTERY = 1,
    DISPLAY_INFO_NUM
} display_info_e;


static raw_adc_samples_t raw_adc_samples = {0};
static debouncer_t temperature_control_debouncer = {.detected_state = false, .change_ticks = 10};
static int32_t setpoint_samples[NUMBER_OF_SETPOINT_SAMPLES] = {0};
static int32_t temperature_samples[NUMBER_OF_TEMPERATURE_SAMPLES] = {0};
static moving_average_filter_t setpoint_filter = {0};
static moving_average_filter_t temperature_filter = {0};
static ema_filter_t battery_voltage_filter = {.alpha_q8 = 200};
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] = {0};
static aircon_status_t aircon_info = {0};
static configuration_data_t configuration_data = {0};
static display_info_e display_screen = DISPLAY_INFO_TEMPERATURE;


static inline void relay_gpio_init(void)
{
    CONFIG_PIN(DDRB, RELAY_1 | RELAY_4 | RELAY_6 | RELAY_7);
    CONFIG_PIN(DDRD, RELAY_2 | RELAY_5 | RELAY_3);
}


void MAIN_test_relays(void)
{
    SET_OUTPUT_HIGH(DDRB, RELAY_1 | RELAY_4 | RELAY_6 | RELAY_7);
    SET_OUTPUT_HIGH(DDRD, RELAY_2 | RELAY_5 | RELAY_3);
    _delay_ms(1000);
    SET_OUTPUT_LOW(DDRB, RELAY_1 | RELAY_4 | RELAY_6 | RELAY_7);
    SET_OUTPUT_LOW(DDRD, RELAY_2 | RELAY_5 | RELAY_3);
}


void MAIN_get_aircon_info(message_aircon_status_t *info)
{
    info->enabled = aircon_info.enabled;
    info->running = aircon_info.run;
    info->state = aircon_info.state;
    info->run_time = aircon_info.run_time;
}


void MAIN_get_data_info(message_adc_average_t *info)
{
    info->temperature = temperature_filter.result;
    info->setpoint = setpoint_filter.result;
    info->battery_voltage = battery_voltage_filter.average_q8;
}


static inline void button_gpio_init(void)
{
    SET_OUTPUT_LOW(DDRD, _BV(AIRCON_ENABLED_BUTTON) | _BV(NEXT_SCREEN_BUTTON));
}


static inline void gpio_init(void)
{
    relay_gpio_init();
    button_gpio_init();
    // LED init
    CONFIG_PIN(DDRD, STATUS_LED);
}


static void task_status_led(void)
{
    // TOGGLE_PIN(PORTD, STATUS_LED);
}


static int display_fixed_point_as_float(int32_t value, uint8_t q_value, uint8_t x_pos,
                                        uint8_t y_pos)
{
    uint8_t bcd[3] = {0};
    int_to_bcd((value * 10) / (1 << q_value), bcd);

    return SSD1306_draw_from_flash(font_15x18_numbers[bcd[0]], sizeof(font_15x18_0), x_pos + 25,
                                   y_pos, 3) ||
           SSD1306_draw_from_flash(font_15x18_numbers[bcd[1]], sizeof(font_15x18_0), x_pos + 50,
                                   y_pos, 3) ||
           SSD1306_draw_from_flash(font_15x18_numbers[bcd[2]], sizeof(font_15x18_0), x_pos + 70,
                                   y_pos, 3);
}


static int display_temperature_info(void)
{
    static uint8_t i = 0;

    uint8_t bcd[3] = {0};
    int_to_bcd((setpoint_filter.result * 10) / (1 << 8), bcd);

    uint8_t x_pos = 20;
    int err =
        SSD1306_draw_from_flash(font_6x7_numbers[bcd[0]], sizeof(font_6x7_dot), x_pos + 15, 0, 0) ||
        SSD1306_draw_from_flash(font_6x7_numbers[bcd[1]], sizeof(font_6x7_dot), x_pos + 30, 0, 0) ||
        SSD1306_draw_from_flash(font_6x7_numbers[bcd[2]], sizeof(font_6x7_dot), x_pos + 39, 0, 0);

    if (aircon_info.state == AIRCON_ON)
    {
        i ^= 1;
        err = err || SSD1306_draw_from_flash(ice_frames[i], sizeof(ice_1), 100, 0, 3);
    }

    return err;
}


static void init_display(void)
{
    TWI_init(19, TWI_DIV_1);
    SSD1306_init(32);

    SSD1306_display_on(false);
    SSD1306_draw_from_flash(degrees_symbol, sizeof(degrees_symbol), 20, 0, 0);
    SSD1306_draw_from_flash(font_6x7_dot, sizeof(font_6x7_dot), 42, 0, 0);
    SSD1306_draw_from_flash(dot_big, sizeof(dot_big), 43, 1, 3);
    SSD1306_draw_from_flash(ice_frames[0], sizeof(ice_1), 100, 0, 3);
}

static void update_display(bool wake_up)
{
    int err = 0;
    static int8_t sleep_timer = -1;

    if (wake_up)
    {
        if (sleep_timer <= 0)
        {
            SSD1306_display_on(true);
            display_screen = DISPLAY_INFO_TEMPERATURE;
        }

        sleep_timer = configuration_data.display_sleep_timer;
    }
    else if (sleep_timer <= -1)
    {
        return;
    }
    else if (sleep_timer == 0)
    {
        SSD1306_display_on(false);
        return;
    }

    switch (display_screen)
    {
    case DISPLAY_INFO_TEMPERATURE:
        err = SSD1306_draw_from_flash(degrees_big, sizeof(degrees_big), 0, 1, 3) ||
              display_temperature_info() ||
              display_fixed_point_as_float(temperature_filter.result, 8, 0, 1);
        break;

    case DISPLAY_INFO_BATTERY:
        err = SSD1306_draw_from_flash(voltage_symbol, sizeof(voltage_symbol), 0, 1, 2) ||
              display_temperature_info() ||
              display_fixed_point_as_float(battery_voltage_filter.average_q8, 8, 0, 1);
        break;

    default:
        break;
    }

    if (err)
    {
        // TODO Fix recovery
        // SET_OUTPUT_HIGH(PORTD, STATUS_LED);
        // for (int i = 0; i < 9; i++)
        // {
        //     TOGGLE_PIN(PORTC, PC5);
        //     _delay_us(10);
        // }

        init_display();
    }

    sleep_timer--;
}

static void task_update_display(void)
{
    update_display(false);
}


static void task_get_adc_readings(void)
{
    ADC_set_channel(0);
    ADC_START_CONVERSION();

    do
    {
        ADC_wait_for_conversion();
    } while (!ADC_sequencer());

    MOVING_AVERAGE_FILTER_update(&temperature_filter,
                                 TO_Q8_TEMPERATURE(raw_adc_samples.temperature));
    MOVING_AVERAGE_FILTER_update(&setpoint_filter, TO_Q8_TEMPERATURE(raw_adc_samples.setpoint));
    EMA_FILTER_update(&battery_voltage_filter, TO_Q8_VOLTAGE(raw_adc_samples.battery_voltage));
}


static void task_update_temperature_control(void)
{
    aircon_info.run = debouncer(&temperature_control_debouncer,
                                temperature_filter.result > setpoint_filter.result);


    if (!aircon_info.enabled)
    {
        aircon_info.state = AIRCON_OFF;
        aircon_info.run = false;
    }

    switch (aircon_info.state)
    {
    case AIRCON_OFF:
    {
        aircon_info.run_time = 0;
        SET_OUTPUT_LOW(PORTB, RELAY_1);
        if (aircon_info.run)
        {
            aircon_info.state = AIRCON_ON;
        }

        break;
    }

    case AIRCON_ON:
    {
        aircon_info.run_time++;
        SET_OUTPUT_HIGH(PORTB, RELAY_1);

        if (!aircon_info.run)
        {
            aircon_info.state = AIRCON_OFF;
        }
        else if (aircon_info.run_time >= configuration_data.safe_continuous_run_time)
        {
            aircon_info.run_time = configuration_data.cooldown_time;
            aircon_info.state = AIRCON_COOLDOWN;
        }

        break;
    }

    case AIRCON_COOLDOWN:
    {
        aircon_info.run_time--;
        SET_OUTPUT_LOW(PORTB, RELAY_1);

        if (aircon_info.run_time == 0)
        {
            aircon_info.state = AIRCON_OFF;
        }
        else if (!aircon_info.run)
        {
            aircon_info.state = AIRCON_OFF;
        }

        break;
    }

    default:
    {
        SET_OUTPUT_LOW(PORTB, RELAY_1);
        break;
    }
    }
}


static void task_get_button_status(void)
{
    aircon_info.enabled = !READ_PIN(PIND, AIRCON_ENABLED_BUTTON);

    if (!READ_PIN(PIND, NEXT_SCREEN_BUTTON))
    {
        display_screen++;
        display_screen = display_screen >= DISPLAY_INFO_NUM ? 0 : display_screen;
        SSD1306_draw_rectangle(23, 0, 0);
        update_display(true);
    }
}

static void load_config(void)
{
    if (PERSISTENT_DATA_load(&configuration_data))
    {
        battery_voltage_filter.alpha_q8 = configuration_data.battery_filter_alpha;
    }
    else
    {
        battery_voltage_filter.alpha_q8 = 200;
    }
}

int main(void)
{
    UART_init(uart_rx_buffer, 51);
    load_config();

    MOVING_AVERAGE_FILTER_init(&setpoint_filter, setpoint_samples, NUMBER_OF_TEMPERATURE_SAMPLES);
    MOVING_AVERAGE_FILTER_init(&temperature_filter, temperature_samples,
                               NUMBER_OF_TEMPERATURE_SAMPLES);

    gpio_init();

    const adc_channel_config_t adc_sequencer[] = {
        {.channel = TEMPERATURE_ADC, .result_ptr = &raw_adc_samples.temperature},
        {.channel = SETPOINT_ADC, .result_ptr = &raw_adc_samples.setpoint},
        {.channel = BATTERY_ADC, .result_ptr = &raw_adc_samples.battery_voltage}};
    ADC_sequencer_init(adc_sequencer, ARRAY_LEN(adc_sequencer));
    ADC_init(false);

    init_display();

    sei(); // Enable interrupts

    task_t tasks[] = {
        {.task_fn = task_status_led, .period = 500, .elapsed_time = 20},
        {.task_fn = task_get_adc_readings, .period = 100, .elapsed_time = 0},
        {.task_fn = task_update_display, .period = 1000, .elapsed_time = 63},
        {.task_fn = task_update_temperature_control, .period = 1000, .elapsed_time = 121},
        {.task_fn = MSG_HANDLER_serial_interface, .period = 1, .elapsed_time = 5},
        {.task_fn = task_get_button_status, .period = 100, .elapsed_time = 98}};

    while (true)
    {
        TASK_SCHEDULER_update(tasks, ARRAY_LEN(tasks));
        _delay_ms(1);
    }

    return 0;
}
