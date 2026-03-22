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
 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/

 typedef enum {
    A_0 = 0,
    A_1,
    A_2,
    A_3,
    A_4,
    A_5,
} ANALOG_PINS;

typedef enum{
    output = 0,
    input = 1
} IO;

 /******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/

void ADC_INIT(void);
void analog_i_o(ANALOG_PINS pins, IO io); // edit pin a to state b 1 is output
int read_analogHL(ANALOG_PINS pin); // reads analog pin as a digital input high is 0.> 6 *VCC low is <0.33*VCC
float read_analog_ADC(ANALOG_PINS pin); // returns a voltage
void write_analog(ANALOG_PINS pin); // write a high low to analog pin