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

 typedef enum{
    pin0 = 0,
    pin1,
    pin2,
    pin3,
    pin4,
    pin5,
    pin6,
    pin7,
    pin8,
    pin9,
    pin10,
    pin11,
    pin12,
    pin13
} digital_pin;

typedef enum{
    off = 0,
    on,
} pullup_status;

typedef enum{
    output = 0,
    input = 1
} IO;

typedef enum{
    LOW_LEVEL= 0,
    ANY_CHANGE,
    FALLING_EDGE,
    RISING_EDGE 
}   interrupt_mode;

typedef enum{
    PIN_D2 = 0,
    PIN_D3
} ISR_Pin;

  /******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
 void digital_i_o(digital_pin pin, IO io); // edit pin a to state b 1 is output
void set_digitalHIGH(digital_pin pin); 
void set_digitalLOW(digital_pin pin);
bool read_digital(digital_pin pin);
void digital_pullup(digital_pin pin, pullup_status pull);
bool init_fastPWM(long int hz, int duty, digital_pin pin); // pin # (9-11) duty cycle is a percentage returns false if frequency is too high
bool edit_PWM(digital_pin pin , int duty,long int hz);
void pwm1_stop(void);
void pwm1_start(void);