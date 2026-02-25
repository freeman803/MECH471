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
}

 void digital_i_o(int a, bool b){
if(b){
    DDRB |= BIT(a);
}
else{
    DDRB &= ~BIT(a);
}
}

void set_digital(int a, bool b){
if(b){
    PORTD |= BIT(a);
}
else{
    PORTD &= ~BIT(a);
}
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


/******************************************************************************
 *                           P U B L I C  V A R S
 ******************************************************************************/
