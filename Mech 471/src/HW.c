/**
 * @file HW.c
 * @brief Module source that defines HW functions
 */

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include "HW.h"

/******************************************************************************
 *                         P R I V A T E  V A R S
 ******************************************************************************/

/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
void HW_init(void){
ADCSRA |= BIT(7); // ADC voltage reference is gnd and not one of the other adc pins
ADCSRA |= BIT(0);//
ADCSRA |= BIT(1);//
ADCSRA |= BIT(2);// top 3 lines set adc pre scaler /128
ADMUX &= ~BIT(5); //adc left adjust result 
//init timers
TCCR1A = 0;
TCCR1B = 0;
}

 void digital_i_o(int a, bool b){
if(b){
    DDRB |= BIT(a);
}
else{
    DDRB &= ~BIT(a);
}
}

void set_digitalHIGH(int a){
    PORTD |= BIT(a);  
}

void set_digitalLOW(int a){
    PORTD &= ~BIT(a);
}

bool read_digital(int a){
return PIND & BIT(a);
}

void analog_i_o(int a, bool b){
if (b){
    DDRC |= BIT(a);
}
else{
    DDRC &= ~BIT(a);
}
}

int read_analogHL(int a){
return PINC & BIT(a);
}

float read_analog_ADC(int a){

ADCSRA |= BIT(6); // start ADC conversion
while (ADCSRA & BIT(4));
//must read adcl before adch
if (a< 8){
return (ADCL&BIT(a)); 
}
if (a>8){
    return (ADCH*BIT(a));
}
}

void write_analog(int a){
    PORTC |= BIT(a);
}
void digital_pullup(int a, bool b){
if(b){
    PORTD |= BIT(a);
}
else{
    PORTD &= ~BIT(a);
}
}

bool init_fastPWM(long int hz, int duty, int pin){
    if (pin > 10 || pin < 8) return false;
    pin = pin -8;
    DDRB |= BIT(pin);
    // look at table 14.8
    TCCR1B |= BIT(3) | BIT(4); // sets wgm02 = 1
    TCCR1A |= BIT(1); // sets wgm00 = 1 and wgm01 = 1
    TCCR1A |= BIT(7);// non inverting input
    TCCR1B |= BIT(0) | BIT(1); // Prescaler = 64
    uint32_t top = (16000000UL / (64UL * hz)) - 1;
    if (top > 65535) return false;
     ICR1 = top;
    OCR1A = (top*duty)/100;
    return true;
}

/******************************************************************************
 *                           P U B L I C  V A R S
 ******************************************************************************/
