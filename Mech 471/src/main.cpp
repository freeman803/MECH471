 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/
#include <Arduino.h>
#define BIT(a) (1UL << (a))
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/

void setup() {
    sei(); //enable global interuupts
//clear bits
ADMUX = 0;
ADCSRA =0;
//select vref i want Aref so bits need to be 00 i just cleared them so they are 0
//since not specified i will read adc0 
ADCSRA |= BIT(ADEN);//enable adc
ADCSRA |= BIT(ADIE);//enable intrupts
//leave prescaler as 2 since not specified
//ADC NOISE REDUCTION MODE
SMCR |= BIT(SM0);//this also starts a ADC conversion ". If the ADC is enabled, a
//conversion starts automatically when this mode is entered."
SMCR |= BIT(SE);//this allows the sleep instruction to enter sleep should explicit off when using prvent bit flip
__asm__ __volatile__("sleep");//inseted asmebly instruction declared it as volatile so compiler wont touch then inserted assembly command from datasheet page 59
}

void loop() {

}
ISR(ADC_vect){
SMCR &= ~BIT(SE);
//do something i guess
}

/*
A) done
B) it wakes up preforms the interrupt then you have to request sleep again
C) the switch will create noise and those are close to the adc traces

*/