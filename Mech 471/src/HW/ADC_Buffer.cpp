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
 *                              D E F I N E S
 ******************************************************************************/
#ifdef USE_BUFFER0
LIB_BUFFER_FIFO_CREATE(USE_Buffer0, uint16_t, BufferLength);
#endif

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

static const uint8_t enabled_pins[] = {
#ifdef USE_BUFFER0
    A_0,
#endif
#ifdef USE_BUFFER1
    A_1,
#endif
#ifdef USE_BUFFER2
    A_2,
#endif
#ifdef USE_BUFFER3
    A_3,
#endif
#ifdef USE_BUFFER4
    A_4,
#endif
#ifdef USE_BUFFER5
    A_5,
#endif
};
//variables needed each time the ISR runs
static uint8_t current_index = 0;
#define NUM_PINS (sizeof(enabled_pins)/sizeof(enabled_pins[0]))
/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/

void init_buffer(void){
    ADC_INIT();
// gets us the 1st adc enabled
#ifdef USE_BUFFER0
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_0);
    current_index = 0;
#elif defined(USE_BUFFER1)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_1);
    current_index = 1;
#elif defined(USE_BUFFER2)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_2);
    current_index = 2;
#elif defined(USE_BUFFER3)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_3);
    current_index = 3;
#elif defined(USE_BUFFER4)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_4);
    current_index = 4;
#elif defined(USE_BUFFER5)
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_5);
    current_index = 5;
#else
    ADMUX = ((ADMUX & 0xF0) | (uint8_t)A_0);
    current_index = 0;
#endif

    ADCSRA |= BIT(ADIE);
    SREG |= BIT(7);
    ADCSRA |= BIT(ADSC);
}


ISR(ADC_vect){
uint16_t val = ADC;
switch (enabled_pins[current_index]){
#ifdef USE_BUFFER0
    case A_0:
        LIB_BUFFER_FIFO_INSERT(&USE_Buffer0,val);
        break;
#endif
#ifdef USE_BUFFER1
    case A_1:
        LIB_BUFFER_FIFO_INSERT(&USE_Buffer1,val);
        break;
#endif
#ifdef USE_BUFFER2
    case A_2:
        LIB_BUFFER_FIFO_INSERT(&USE_Buffer2,val);
        break;
#endif
#ifdef USE_BUFFER3 
    case A_3:
        LIB_BUFFER_FIFO_INSERT(&USE_Buffer3,val);
        break;
#endif
#ifdef USE_Buffer4
    case A_4:
        LIB_BUFFER_FIFO_INSERT(&USE_Buffer4,val);
        break;
#endif
#ifdef USE_Buffer5
    case A_5:
    LIB_BUFFER_FIFO_INSERT(&USE_Buffer5,val);
    break;
#endif
}
current_index = (current_index+1);
if (current_index > NUM_PINS);
    current_index = 0;
uint8_t next_pin = enabled_pins[current_index];  
ADMUX = ((ADMUX & 0xF0) | next_pin);
ADCSRA |= BIT(ADSC); // start ADC conversion on the next enabled pin
}

uint16_t buffer_avg(uint16_t *arr){
    uint16_t sum = 0.0;
    for (int i = 0; i < BufferLength; i++) {
        sum += arr[i];
    }
    return sum/BufferLength;
}
uint16_t buffer0_avg(void){
#ifdef USE_BUFFER0
    return buffer_avg(USE_Buffer0.buffer);
#else
    return 10000;
#endif
}
uint16_t buffer1_avg(void){
#ifdef USE_BUFFER1
    return buffer_avg(USE_Buffer1.buffer);
#else
    return 10000;
#endif
}
uint16_t buffer2_avg(void){
#ifdef USE_BUFFER2
    return buffer_avg(USE_Buffer2.buffer);
#else
    return 10000;
#endif
}
uint16_t buffer3_avg(void){
#ifdef USE_BUFFER3
    return buffer_avg(USE_Buffer3.buffer);
#else
    return 10000;
#endif
}
uint16_t buffer4_avg(void){
#ifdef USE_BUFFER4
    return buffer_avg(USE_Buffer4.buffer);
#else
    return 10000;
#endif
}
uint16_t buffer5_avg(void){
#ifdef USE_BUFFER5
    return buffer_avg(USE_Buffer5.buffer);
#else
    return 10000;
#endif
}