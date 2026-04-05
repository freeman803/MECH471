/**
 * @file HW.h
 * @brief This will be the file defines our register level code
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
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
//#define USE_BUFFER0
#define USE_BUFFER1
//#define USE_BUFFER2
//#define USE_BUFFER3
//#define USE_BUFFER4
//#define USE_BUFFER5
#endif

 /******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
uint16_t buffer_avg(uint16_t *arr);
uint16_t buffer1_avg(void);
uint16_t buffer2_avg(void);
uint16_t buffer3_avg(void);
uint16_t buffer4_avg(void);
uint16_t buffer5_avg(void);
uint16_t buffer6_avg(void);
uint16_t read_analog_buffer(ANALOG_PINS pin);
void init_buffer(void);
