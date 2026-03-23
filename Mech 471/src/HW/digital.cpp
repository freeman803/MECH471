/**
 * @file ADC.cpp
 * @brief Module source that defines ADC functions
 */

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <HW\digital.h>

/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
 
 void digital_i_o(digital_pin pin, IO io){
if(io){
    DDRD &= ~BIT(pin); //input
    PORTB &= ~BIT(pin);  // no pull-up
}
else{
    DDRD |= BIT(pin); //output
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

void digital_pullup(digital_pin pin, pullup_status pull){
if(pull){
    PORTD |= BIT(pin);
}
else{
    PORTD &= ~BIT(pin);
}
}

bool init_fastPWM(long int hz, int duty, digital_pin pin){
    // Stop timer completely first
    TCCR1B = 0;
    TCCR1A = 0;
    TCNT1  = 0;  // Reset counter
    ICR1   = 0;
    OCR1A  = 0;
    OCR1B  = 0;
    if (pin != pin10 ) return false;

    if (pin == pin9) DDRB |= BIT(DDB1);
    if (pin == pin10) DDRB |= BIT(DDB2);

    // look at table 14.8
    TCCR1B |=  BIT(WGM13) | BIT(WGM12);
    TCCR1A |=  BIT(WGM11);
    TCCR1A &= ~BIT(WGM10);
    //normal operation
    TCCR1A &= ~(BIT(COM1A1) | BIT(COM1A0) | BIT(COM1B1) | BIT(COM1B0));
    if (pin == pin9)  TCCR1A |= BIT(COM1A1);
    if (pin == pin10) TCCR1A |= BIT(COM1B1);
    
     uint32_t top = (16000000UL / (64UL * hz)) - 1;
    if (top > 65535) return false;
     ICR1 = (uint16_t)top;
    if (pin == pin9)
        OCR1A = (top * duty) / 100;
    else if (pin == pin10)
        OCR1B = (top * duty) / 100;
    else 
        return false;
    // Prescaler = 64
    TCCR1B &= ~BIT(CS12);
    TCCR1B |= BIT(CS10) | BIT(CS11); 
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

void pwm1_start(void)
{
    // Prescaler = 64 (same as your init)
    TCCR1B |= BIT(0) | BIT(1);
}

void pwm1_stop(void)
{
    // Clear clock select bits (CS12:CS10)
    TCCR1B &= ~(BIT(0) | BIT(1) | BIT(2));
}