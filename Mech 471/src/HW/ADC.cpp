/**
 * @file ADC.cpp
 * @brief Module source that defines ADC functions
 */

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <HW\ADC.h>

/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
void ADC_INIT(void){
    ADCSRA |= BIT(ADEN);//enable ADC
    //divide by 2 (prescaler)
    ADCSRA |= BIT(ADPS0);//
    ADCSRA |= BIT(ADPS1);//
    ADCSRA |= BIT(ADPS2);//
    //no auto trigger
    ADMUX &= ~BIT(ADLAR); // this does a right adjust 
    //internal voltage refs
    ADMUX |= BIT(REFS0);
    ADMUX &= ~BIT(REFS1);

}
void analog_i_o(ANALOG_PINS pins, IO io){
if (io){
    DDRC &= ~BIT(pins);
    PORTC &= ~BIT(pins); 
}
else{
    DDRC |= BIT(pins);
}
}
int read_analogHL(ANALOG_PINS pin){
return PINC & BIT(pin);
}
float read_analog_ADC(ANALOG_PINS pin){
    if (pin > 6) 
        return 0.0; // check for invalid input
    //to select which pin
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)pin);
    ADCSRA |= BIT(ADSC); // start ADC conversion
    while (ADCSRA & BIT(ADSC));
    //must read adcl before adch
    uint16_t result = ADC;
    return result *(5.0/1023.0);
}
void write_analog(ANALOG_PINS pin){
    PORTC |= BIT(pin);
}