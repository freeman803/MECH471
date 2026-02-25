/**
 * @file HW.h
 * @brief This will be the file defines our register level code
 */

 #include <Arduino.h>
 #include <avr/io.h> 
 #define BIT(a) (1UL << (a))


/******************************************************************************
 *                             P U B L I C  F U N C T I O N S
 ******************************************************************************/
// HW INIT
void HW_init(void);

// digtal pin input or ouput
//pin 8 = bit 5
void digital_i_o(int a, bool b); // edit pin a to state b 1 is output
void set_digital(int a, bool b); // edit pin a to state b, 1 is high 
bool read_digital(int a);
void set_PWM(int a);

//ANALOG PINS
void analog_i_o(int a, bool b); // edit pin a to state b 1 is output
int read_analogHL(int a); // reads analog pin as a digital input high is 0.> 6 *VCC low is <0.33*VCC
float read_analog_ADC(int a); // returns a voltage
