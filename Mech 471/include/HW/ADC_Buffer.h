/**
 * @file HW.h
 * @brief This will be the file defines our register level code
 */
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/

 #include <Arduino.h>
 #include <avr/io.h> 
 #include<HW/define.h>
 #include <HW\ADC.h>
 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/
#define BufferLength ((uint8_t)50)


#ifndef FIFO_CONFIG_H
#define FIFO_CONFIG_H

// Uncomment the buffers you actually need
//#define USE_BUFFER1
//#define USE_BUFFER2
//#define USE_BUFFER3
//#define USE_BUFFER4
//#define USE_BUFFER5
//#define USE_BUFFER6

#endif

 /******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
float buffer_avg(float (*arr)[BufferLength]);