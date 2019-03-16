/*
    wykys 2019
    PWM fan regulator for expand RAMPS 1.4 wCUBE board
*/

#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define PIN_PWM_BOX PB0
#define PIN_PWM_HEAD PB1
#define PIN_ADC_BOX PB4
#define PIN_ADC_HEAD PB3
#define PIN_ONOFF PB2

#define OCR_BOX OCR0A
#define OCR_HEAD OCR0B

#define ADC_CHANNEL_BOX 3
#define ADC_CHANNEL_HEAD 2
#define ADC_CHANNEL_DEFAULT ADC_CHANNEL_HEAD
#define ADC_SAMPLES_LEN 13

FUSES = {
    .low = 0x3A,
    .high = 0xFF,
};

void gpio_init(void)
{
    DDRB = (1 << PIN_PWM_BOX) | (1 << PIN_PWM_HEAD);
    PORTB = (1 << PIN_ONOFF);
}

void timer_init(void)
{
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | // clear OC0x on compare match, set OC0x at TOP
             (1 << WGM01) | (1 << WGM00);    // mode 3 fast PWM
    TCCR0B = (1 << CS00);                    // clk = F_CPU
    TIMSK0 = (1 << TOIE0);                   // enable overflow interrupt
    TCNT0 = 0;                               // clear timer
    OCR_BOX = 1;
    OCR_HEAD = 0xFF;
}

void adc_init(void)
{
    ADMUX = (1 << ADLAR);               // left align from h byte bits 9-2 in ADCH
    ADMUX |= ADC_CHANNEL_DEFAULT;       // set default channel
    ADCSRA = (1 << ADEN) | (1 << ADIE); // enable ADC, adc_clk = F_CPU/2, enable interrupt
    ADCSRB = (1 << ADTS2);              // trigger on timer overflow
    ADCSRA |= (1 << ADATE);             // start conversion with trigger
}

ISR(TIM0_OVF_vect)
{
    // this interrupt is trigger for ADC
}

ISR(ADC_vect)
{
    uint16_t average;
    uint8_t channel = ADMUX & ((1 << MUX1) | (1 << MUX0));
    static uint8_t sample[ADC_SAMPLES_LEN] = {0};
    static uint8_t i = 0;

    sample[i++] = ADCH;

    if (i == ADC_SAMPLES_LEN)
    {
        ADCSRA &= ~(1 << ADEN);

        for (average = 0, i = 0; i < ADC_SAMPLES_LEN; i++)
            average += sample[i] * 10;
        average /= ADC_SAMPLES_LEN * 10;
        average = (bit_is_set(PINB, PIN_ONOFF)) ? average : 0;

        if (channel == ADC_CHANNEL_HEAD)
        {
            OCR_HEAD = average;
            channel = ADC_CHANNEL_BOX;
        }
        else
        {
            OCR_BOX = average;
            channel = ADC_CHANNEL_HEAD;
        }

        ADMUX &= ~((1 << MUX1) | (1 << MUX0));
        ADMUX |= channel;
        ADCSRA |= (1 << ADEN);

        i = 0;
    }
}

int main(void)
{
    gpio_init();
    adc_init();
    timer_init();
    sei();

    while (1)
        ;

    return 0;
}