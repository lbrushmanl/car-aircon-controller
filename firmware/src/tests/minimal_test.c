
#include "utils.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

// LED Mapping
// PD2
// Status
#define STATUS_LED _BV(PD2)


int main(void)
{
    CONFIG_PIN(DDRD, STATUS_LED);

    while (true)
    {
        TOGGLE_PIN(PORTD, STATUS_LED);
        _delay_ms(250);
    }

    return 0;
}
