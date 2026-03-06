/**
 * @file HW.h
 * @brief This will be the file defines our register level code
 */
/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/

 #include <Arduino.h>
 #include <avr/io.h> 

 /******************************************************************************
 *                              D E F I N E S
 ******************************************************************************/
 #define BIT(a) (1UL << (a))
//  | Interrupt   | Real vector   |
// | ------------ | ------------- |
// | INT0         | `__vector_1`  |
// | INT1         | `__vector_2`  |
// | PCINT0       | `__vector_3`  |
// | TIMER1 COMPA | `__vector_11` |
#define MY_ISR(vector) void __attribute__((signal, used)) vector(void)
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
typedef enum {
    A_0 = 0,
    A_1,
    A_2,
    A_3,
    A_4,
    A_5,
} ANALOG_PINS;



/******************************************************************************
 *                             P U B L I C  F U N C T I O N S
 ******************************************************************************/
// HW INIT
void HW_init(void);

// digtal pin input or ouput
//pin 8 = bit 5
void digital_i_o(int a, bool b); // edit pin a to state b 1 is output
void set_digitalHIGH(int a); // edit pin a to state b, 1 is high 
void set_digitalLOW(int a);
bool read_digital(int a);
void digital_pullup(int a, bool b); //pull up on b = 1
bool init_fastPWM(long int hz, int duty, int pin); // pin # (9-11) duty cycle is a percentage returns false if frequency is too high

//ANALOG PINS
void analog_i_o(int a, bool b); // edit pin a to state b 1 is output
int read_analogHL(int a); // reads analog pin as a digital input high is 0.> 6 *VCC low is <0.33*VCC
float read_analog_ADC(ANALOG_PINS pin); // returns a voltage
void write_analog(int a); // write a high low to analog pin

//ISR Functions
bool define_ISR(interrupt_mode mode, ISR_Pin pin);//   returns false if did not set ISR
