/**
 * @file ADC.cpp
 * @brief Module source that defines ADC functions
 */


 // ADCSRA |= BIT(ADIE);   // Enable ADC interrupt toggling this pin will enable the ISR
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <HW\ADC.h>
#include <HW\ADC_Buffer.h>
#include <avr/interrupt.h>
#include "lib_buffer.h"
/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
// buffer1 -> ADC1
// buffer2 -> ADC2
// buffer3 -> ADC3
// buffer4 -> ADC4
// buffer5 -> ADC5
void init_buffer(void){
    ADC_INIT();

#ifdef USE_BUFFER1
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_0);
#elif defined(USE_BUFFER2)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_1);
#elif defined(USE_BUFFER3)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_2);
#elif defined(USE_BUFFER4)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_3);
#elif defined(USE_BUFFER5)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_4);
#elif defined(USE_BUFFER6)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_5);
#endif

    ADCSRA |= BIT(ADIE);
    SREG |= BIT(7);
    ADCSRA |= BIT(ADSC);
}

uint16_t read_analog_buffer(ANALOG_PINS pin){
    uint16_t result = ADC;
    uint8_t next_pin = ADMUX & 0x0F; // this gives current pin

    switch (next_pin) {
#ifdef USE_BUFFER1
        case A_0:
#if defined(USE_BUFFER2)
            next_pin = A_1;
#elif defined(USE_BUFFER3)
            next_pin = A_2;
#elif defined(USE_BUFFER4)
            next_pin = A_3;
#elif defined(USE_BUFFER5)
            next_pin = A_4;
#elif defined(USE_BUFFER6)
            next_pin = A_5;
#else
            next_pin = A_0;
#endif
            break;
#endif
#ifdef USE_BUFFER2
        case A_1:
#if defined(USE_BUFFER3)
            next_pin = A_2;
#elif defined(USE_BUFFER4)
            next_pin = A_3;
#elif defined(USE_BUFFER5)
            next_pin = A_4;
#elif defined(USE_BUFFER6)
            next_pin = A_5;
#elif defined(USE_BUFFER1)
            next_pin = A_0;
#else
            next_pin = A_1;
#endif
            break;
#endif
#ifdef USE_BUFFER3
        case A_2:
#if defined(USE_BUFFER4)
            next_pin = A_3;
#elif defined(USE_BUFFER5)
            next_pin = A_4;
#elif defined(USE_BUFFER6)
            next_pin = A_5;
#elif defined(USE_BUFFER1)
            next_pin = A_0;
#elif defined(USE_BUFFER2)
            next_pin = A_1;
#else
            next_pin = A_2;
#endif
            break;
#endif
#ifdef USE_BUFFER4
        case A_3:
#if defined(USE_BUFFER5)
            next_pin = A_4;
#elif defined(USE_BUFFER6)
            next_pin = A_5;
#elif defined(USE_BUFFER1)
            next_pin = A_0;
#elif defined(USE_BUFFER2)
            next_pin = A_1;
#elif defined(USE_BUFFER3)
            next_pin = A_2;
#else
            next_pin = A_3;
#endif
            break;
#endif
#ifdef USE_BUFFER5
        case A_4:
#if defined(USE_BUFFER6)
            next_pin = A_5;
#elif defined(USE_BUFFER1)
            next_pin = A_0;
#elif defined(USE_BUFFER2)
            next_pin = A_1;
#elif defined(USE_BUFFER3)
            next_pin = A_2;
#elif defined(USE_BUFFER4)
            next_pin = A_3;
#else
            next_pin = A_4;
#endif
            break;
#endif
#ifdef USE_BUFFER6
        case A_5:
#if defined(USE_BUFFER1)
            next_pin = A_0;
#elif defined(USE_BUFFER2)
            next_pin = A_1;
#elif defined(USE_BUFFER3)
            next_pin = A_2;
#elif defined(USE_BUFFER4)
            next_pin = A_3;
#elif defined(USE_BUFFER5)
            next_pin = A_4;
#else
            next_pin = A_5;
#endif
            break;
#endif
        default:
#ifdef USE_BUFFER1
            next_pin = A_0;
#elif defined(USE_BUFFER2)
            next_pin = A_1;
#elif defined(USE_BUFFER3)
            next_pin = A_2;
#elif defined(USE_BUFFER4)
            next_pin = A_3;
#elif defined(USE_BUFFER5)
            next_pin = A_4;
#elif defined(USE_BUFFER6)
            next_pin = A_5;
#endif
            break;
    }

    ADMUX = ((ADMUX & 0xF0) | next_pin);
    ADCSRA |= BIT(ADSC); // start ADC conversion on the next enabled pin
    return result;
}

#ifdef USE_BUFFER1
LIB_BUFFER_FIFO_CREATE(USE_Buffer1, uint16_t, BufferLength);
#endif

#ifdef USE_BUFFER2
LIB_BUFFER_FIFO_CREATE(USE_Buffer2, uint16_t, BufferLength);
#endif

#ifdef USE_BUFFER3
LIB_BUFFER_FIFO_CREATE(USE_Buffer3, uint16_t, BufferLength);
#endif

#ifdef USE_BUFFER4
LIB_BUFFER_FIFO_CREATE(USE_Buffer4, uint16_t, BufferLength);
#endif

#ifdef USE_BUFFER5
LIB_BUFFER_FIFO_CREATE(USE_Buffer5, uint16_t, BufferLength);
#endif

#ifdef USE_BUFFER6
LIB_BUFFER_FIFO_CREATE(USE_Buffer6, uint16_t, BufferLength);
#endif

ISR(ADC_vect){
uint16_t val;
#ifdef USE_BUFFER1
    val = read_analog_buffer(A_0);
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer1, val);
#endif

#ifdef USE_BUFFER2
    val = read_analog_buffer(A_1);
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer2, val);
#endif

#ifdef USE_BUFFER3
    val = read_analog_ADC(A_2);
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer3, val);
#endif

#ifdef USE_BUFFER4
    val = read_analog_buffer(A_3);
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer4, val);
#endif

#ifdef USE_BUFFER5
    val = read_analog_buffer(A_4);
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer5, val);
#endif

#ifdef USE_BUFFER6
    val = read_analog_buffer(A_5);
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer6, val);
#endif

}

uint16_t buffer_avg(uint16_t *arr){
    uint16_t sum = 0.0;
    for (int i = 0; i < BufferLength; i++) {
        sum += arr[i];
    }
    return sum/BufferLength;
}

