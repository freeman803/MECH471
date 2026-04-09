/**
 * @file timer.h
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

struct time_differencePID{
uint32_t last_time=0;
uint32_t time_now=0;
uint32_t dt=1;
};

  /******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
void timer0_init(void);
uint32_t millis_(void);
void update_dt(struct time_differencePID *time);
void init_dt(struct time_differencePID *time);
void delay_ms(uint32_t ms);