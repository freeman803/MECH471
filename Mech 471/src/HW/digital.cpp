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
static volatile uint8_t* ddr_for_pin(digital_pin pin){
    return (pin <= pin7) ? &DDRD : &DDRB;
}

static volatile uint8_t* port_for_pin(digital_pin pin){
    return (pin <= pin7) ? &PORTD : &PORTB;
}

static volatile uint8_t* pin_reg_for_pin(digital_pin pin){
    return (pin <= pin7) ? &PIND : &PINB;
}

static uint8_t bit_for_pin(digital_pin pin){
    return (pin <= pin7) ? (uint8_t)pin : (uint8_t)(pin - pin8);
}

void digital_i_o(digital_pin pin, IO io){
    volatile uint8_t* ddr = ddr_for_pin(pin);
    volatile uint8_t* port = port_for_pin(pin);
    uint8_t bit = bit_for_pin(pin);

    if(io){
        *ddr &= ~BIT(bit); //input
        *port &= ~BIT(bit);  // no pull-up
    }
    else{
        *ddr |= BIT(bit); //output
    }
}

void set_digitalHIGH(digital_pin pin){
    *port_for_pin(pin) |= BIT(bit_for_pin(pin));
}

void set_digitalLOW(digital_pin pin){
    *port_for_pin(pin) &= ~BIT(bit_for_pin(pin));
}

bool read_digital(digital_pin pin){
    return (*pin_reg_for_pin(pin) & BIT(bit_for_pin(pin))) != 0;
}

void digital_pullup(digital_pin pin, pullup_status pull){
    volatile uint8_t* port = port_for_pin(pin);
    uint8_t bit = bit_for_pin(pin);

    if(pull){
        *port |= BIT(bit);
    }
    else{
        *port &= ~BIT(bit);
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
    if (pin != pin10 && pin != pin9) return false;

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
    
    if (hz <= 0 || duty < 0 || duty > 100) return false;
    uint32_t top = (F_CPU / (64UL * (uint32_t)hz)) - 1;
    if (top > 65535UL) return false;
    ICR1 = (uint16_t)top;
    if (pin == pin9) {
        OCR1A = (uint16_t)(((uint32_t)top * (uint32_t)duty) / 100UL);
    } else if (pin == pin10) {
        OCR1B = (uint16_t)(((uint32_t)top * (uint32_t)duty) / 100UL);
    } else {
        return false;
    }
    // Prescaler = 64
    TCCR1B &= ~(BIT(CS12) | BIT(CS11) | BIT(CS10));
    TCCR1B |= BIT(CS11) | BIT(CS10);
    return true;
}

bool edit_PWM(digital_pin pin, int duty, long int hz){
    uint32_t top = (16000000UL / (64UL * (uint32_t)hz)) - 1;
    if (top > 65535UL) return false;
    ICR1 = (uint16_t)top;
    if (pin == pin9) {
        OCR1A = (uint16_t)(((uint32_t)top * (uint32_t)duty) / 100UL);
    } else if (pin == pin10) {
        OCR1B = (uint16_t)(((uint32_t)top * (uint32_t)duty) / 100UL);
    } else {
        return false;
    }
    return true;
}

bool define_ISR(interrupt_mode mode, ISR_Pin pin){
    int adj  = 0;
    int adj1 = 0;
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

void init_servoPWM(void)
{
    // Timing Math:
    // F_CPU = 16 MHz, Prescaler = 8 → Timer clock = 2 MHz → tick = 0.5 µs
    // 20 ms period = 40,000 ticks → ICR1 = 39,999
    // 1000 µs = 2000 ticks (full reverse), 1500 µs = 3000 (neutral), 2000 µs = 4000 (full fwd)

    // set D7 (u1) and D8 (u2) as outputs
    DDRD |= BIT(DDD7);
    DDRB |= BIT(DDB0);

    // initialise both pins LOW
    PORTD &= ~BIT(PORTD7);
    PORTB &= ~BIT(PORTB0);

    cli(); // disable interrupts for atomic Timer1 config

    // clear Timer1 control registers
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;

    // Fast PWM, Mode 14: WGM13=1 WGM12=1 WGM11=1 WGM10=0 — ICR1 is TOP
    // COM1A=00, COM1B=00: no hardware output on D9/D10
    TCCR1A |= BIT(WGM11);
    TCCR1B |= BIT(WGM13) | BIT(WGM12);

    // 20 ms period at 2 MHz
    ICR1  = 39999;

    // neutral pulse widths (1500 µs = 3000 ticks)
    OCR1A = 3000;
    OCR1B = 3000;

    // enable: overflow (start of pulse), compare A (end u1), compare B (end u2)
    TIMSK1 |= BIT(TOIE1) | BIT(OCIE1A) | BIT(OCIE1B);

    // set prescaler /8 last — starts the timer
    TCCR1B |= BIT(CS11);

    sei();
}

void set_u1_pulse(uint16_t us)
{
    // atomic 16-bit write — AVR is 8-bit so both bytes must be written without ISR firing between
    cli();
    OCR1A = (uint16_t)(us * 2);
    sei();
}

void set_u2_pulse(uint16_t us)
{
    cli();
    OCR1B = (uint16_t)(us * 2);
    sei();
}
