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

 void digital_i_o(digital_pin pin, IO io){
if(io){
    DDRB |= BIT(pin);
}
else{
    DDRB &= ~BIT(pin);
}
}

void set_digitalHIGH(digital_pin pin){
    PORTD |= BIT(pin);  
}

void set_digitalLOW(digital_pin pin){
    PORTD &= ~BIT(pin);
}

bool read_digital(digital_pin pin){
return PIND & BIT(pin);
}

void analog_i_o(ANALOG_PINS pins, IO io){
if (io){
    DDRC |= BIT(pins);
}
else{
    DDRC &= ~BIT(pins);
}
}

int read_analogHL(ANALOG_PINS pin){
return PINC & BIT(pin);
}

float read_analog_ADC(ANALOG_PINS pin){
    if (pin > 6) 
        return 0.0; // check for invalid input
    //reference voltage
    ADMUX |= BIT(6);
    ADMUX &= ~BIT(7);
    pin &= 0x0F;
    ADMUX &= ~(0x0F);//clear mux bits
    ADMUX |= pin;
    ADCSRA |= BIT(6); // start ADC conversion
    while (ADCSRA & BIT(6));
    //must read adcl before adch
    uint16_t result = ADCL;
    result |= (uint16_t)ADCH <<8;
    return result *(5.0/1023.0);
}

void write_analog(ANALOG_PINS pin){
    PORTC |= BIT(pin);
}
void digital_pullup(digital_pin pin, pullup_status pull){
if(pull){
    PORTD |= BIT(pin);
}
else{
    PORTD &= ~BIT(pin);
}
}

bool init_fastPWM(long int hz, int duty, digital_pin pin){
    if (pin > pin10 || pin < pin9) return false;
    if (pin == pin9) DDRB |= BIT(1);
    if (pin == pin10) DDRB |= BIT(2);
    // look at table 14.8
    TCCR1B |= BIT(3) | BIT(4); // sets wgm02 = 1
    TCCR1A |= BIT(1); // sets wgm00 = 1 and wgm01 = 1
    // non-inverting
    if (pin == pin9)  TCCR1A |= BIT(7); // COM1A1
    if (pin == pin10) TCCR1A |= BIT(5); // COM1B1
    TCCR1B |= BIT(0) | BIT(1); // Prescaler = 64
    uint32_t top = (16000000UL / (64UL * hz)) - 1;
    if (top > 65535) return false;
     ICR1 = top;
    if (pin == pin9)
        OCR1A = (top * duty) / 100;
    else if (pin == pin10)
        OCR1B = (top * duty) / 100;
    else 
        return false;
    return true;
}

bool edit_PWM(digital_pin pin, int duty, long int hz){
    uint32_t top = (16000000UL / (64UL * hz)) - 1;
    if (top > 65535) return false;
     ICR1 = top;
    if (pin == pin9)
        OCR1A = (top * duty) / 100;
    else if (pin == pin10)
        OCR1B = (top * duty) / 100;
    else 
        return false;
    return true;
}
bool define_ISR(interrupt_mode mode, ISR_Pin pin){
    int adj;
    int adj1;
    if (pin == PIN_D2){
        adj = 0;
        adj1 = 0;
        DDRD &= ~BIT(2);
    }
    else if(pin == PIN_D3){
        adj = 2;
        adj1 = 1;
        DDRD &= ~BIT(3);
    }
    else{
        return false;
    }
    
    switch (mode){
        case LOW_LEVEL: 
            EICRA &= ~BIT(0+adj);
            EICRA &= ~BIT(1+adj);
            break;
        case ANY_CHANGE: 
            EICRA |= BIT(0+adj);
            EICRA &= ~BIT(1+adj);
            break;
        case FALLING_EDGE: 
            EICRA &= ~BIT(0+adj);
            EICRA |= BIT(1+adj);
            break;
        case RISING_EDGE: 
            EICRA |= BIT(0+adj);
            EICRA |= BIT(1+adj);
            break;

        default:
            return false;
    }
    EIFR |= BIT(0+adj1);//clear interrupts
    EIMSK |= BIT(0+adj1);
    SREG |= BIT(7);
    return true;
    }
/******************************************************************************
 *                           P U B L I C  V A R S
 ******************************************************************************/
