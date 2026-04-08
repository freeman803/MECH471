/**
 * @file timer.cpp
 * @brief Module source that defines HW functions
 */

/******************************************************************************
 *                             I N C L U D E S
 ******************************************************************************/
#include <HW\timer.h>

/******************************************************************************
 *                         P R I V A T E  V A R S
 ******************************************************************************/
static volatile uint32_t millis_counter = 0;

/******************************************************************************
 *                       P U B L I C  F U N C T I O N S
 ******************************************************************************/
void timer0_init(void)
{
    //init timers
    TCCR1A = 0;
    TCCR1B = 0;
    // CTC mode
    TCCR0A |= BIT(1);   // WGM01 = 1
    TCCR0A &= ~BIT(0);

    // Prescaler = 64
    TCCR0B |= BIT(0) | BIT(1);
    TCCR0B &= ~BIT(2);

    OCR0A = 249;

    TIMSK0 |= BIT(1); 
}
ISR(TIMER0_COMPA_vect)
{
    millis_counter++;
}
uint32_t millis_(void)
{
    uint32_t m;
    SREG &= ~BIT(7);// disables interrupts so we dont accidentally trigger any isr while doing this this is critical 
    m = millis_counter;
    SREG |= BIT(7); // renables interrupts so we can use them lmao
    return m;
}
void update_dt(struct time_differencePID *time){
    time->last_time = time->time_now;
    time->time_now = millis_();
    time->dt = (time->time_now-time->last_time);
}
void init_dt(struct time_differencePID *time){//this might lead to a huge 1st dt but it should be called pretty soon after boot 
    time->time_now = millis_();
    time->last_time = 0;
    time->dt = time->time_now-time->last_time;
}
void delay_ms(uint32_t ms) {
    uint32_t start = millis_();
    while (abs(millis_() - start) < ms);
}