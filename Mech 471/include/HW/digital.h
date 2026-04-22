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
    off = 0,
    on,
} pullup_status;

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
bool define_ISR(interrupt_mode mode, ISR_Pin pin);
void pwm1_stop(void);
void pwm1_start(void);

// Timer1 interrupt-mode servo pulse generation on D7 (u1) and D8 (u2)
void init_servoPWM(void);
void set_u1_pulse(uint16_t us);   // D7: drive motor, 1000-2000 µs
void set_u2_pulse(uint16_t us);   // D8: steering,    1000-2000 µs
