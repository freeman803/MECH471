/**
 * @file ADC.cpp
 * @brief Module source that defines ADC functions
 */


 // ADCSRA |= BIT(ADIE);   // Enable ADC interrupt toggling this pin will enable the ISR
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <HW\ADC.h>
#include <HW\lib_buffer.h>
#include <HW\ADC_Buffer.h>
/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
// buffer1 -> ADC1
// buffer2 -> ADC2
// buffer3 -> ADC3
// buffer4 -> ADC4
// buffer5 -> ADC5

#ifdef USE_BUFFER1
LIB_BUFFER_FIFO_CREATE(buffer1, float, BufferLength);
#endif

#ifdef USE_BUFFER2
LIB_BUFFER_FIFO_CREATE(buffer2, float, BufferLength);
#endif

#ifdef USE_BUFFER3
LIB_BUFFER_FIFO_CREATE(buffer3, float, BufferLength);
#endif

#ifdef USE_BUFFER4
LIB_BUFFER_FIFO_CREATE(buffer4, float, BufferLength);
#endif

#ifdef USE_BUFFER5
LIB_BUFFER_FIFO_CREATE(buffer5, float, BufferLength);
#endif

#ifdef USE_BUFFER6
LIB_BUFFER_FIFO_CREATE(buffer6, float, BufferLength);
#endif

ISR(ADC_vect){
float val;
#ifdef USE_BUFFER1
    val = read_analog_ADC(A_0);
    LIB_BUFFER_FIFO_INSERT(&buffer1, val);
#endif

#ifdef USE_BUFFER2
    val = read_analog_ADC(A_1);
    LIB_BUFFER_FIFO_INSERT(&buffer2, val);
#endif

#ifdef USE_BUFFER3
    val = read_analog_ADC(A_2);
    LIB_BUFFER_FIFO_INSERT(&buffer3, val);
#endif

#ifdef USE_BUFFER4
    val = read_analog_ADC(A_3);
    LIB_BUFFER_FIFO_INSERT(&buffer4, val);
#endif

#ifdef USE_BUFFER5
    val = read_analog_ADC(A_4);
    LIB_BUFFER_FIFO_INSERT(&buffer5, val);
#endif

#ifdef USE_BUFFER6
    val = read_analog_ADC(A_5);
    LIB_BUFFER_FIFO_INSERT(&buffer6, val);
#endif

}

float buffer_avg(float (*arr)[BufferLength]){
    float sum = 0.0f;
    for (int i = 0; i < BufferLength; i++) {
        sum += (*arr)[i];
    }
    return sum/BufferLength;
}

