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

typedef enum{
    output = 0,
    input = 1
} IO;

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

struct time_differencePID{
uint32_t last_time;
uint32_t time_now;
uint32_t dt;
};
/******************************************************************************
 *                             P U B L I C  F U N C T I O N S
 ******************************************************************************/
// HW INIT
void HW_init(void);

// digtal pin input or ouput
//pin 8 = bit 5
void digital_i_o(digital_pin pin, IO io); // edit pin a to state b 1 is output
void set_digitalHIGH(digital_pin pin); 
void set_digitalLOW(digital_pin pin);
bool read_digital(digital_pin pin);
void digital_pullup(digital_pin pin, pullup_status pull);
bool init_fastPWM(long int hz, int duty, digital_pin pin); // pin # (9-11) duty cycle is a percentage returns false if frequency is too high
bool edit_PWM(digital_pin pin , int duty,long int hz);
void pwm1_stop(void);
void pwm1_start(void);
//ANALOG PINS
void analog_i_o(ANALOG_PINS pins, IO io); // edit pin a to state b 1 is output
int read_analogHL(ANALOG_PINS pin); // reads analog pin as a digital input high is 0.> 6 *VCC low is <0.33*VCC
float read_analog_ADC(ANALOG_PINS pin); // returns a voltage
void write_analog(ANALOG_PINS pin); // write a high low to analog pin

//ISR Functions
bool define_ISR(interrupt_mode mode, ISR_Pin pin);//   returns false if did not set ISR

//Timers
void timer0_init(void);
uint32_t millis_(void);
void update_dt(struct time_differencePID *time);
void init_dt(struct time_differencePID *time);
// these funtions are how you define the interrupts must only be defined once vector1 is D2 vector2 is D3
// MY_ISR(__vector_1) 
// {
// }
// MY_ISR(__vector_2)  
// {
// }