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
void timer2_init(void)
{
    // Reset Timer2 registers
    TCCR2A = 0;
    TCCR2B = 0;

    // CTC mode
    TCCR2A |= BIT(1);   // WGM21 = 1
    TCCR2A &= ~BIT(0);  // WGM20 = 0

    // Prescaler = 64
    TCCR2B |= BIT(2);         // CS22 = 1
    TCCR2B &= ~(BIT(0) | BIT(1)); // CS21=0, CS20=0

    OCR2A = 249;

    TIMSK2 |= BIT(1);   // OCIE2A = 1
}
ISR(TIMER2_COMPA_vect)

{
    millis_counter++;
}
uint32_t millis_(void)
{
    uint32_t m;
    uint8_t sreg = SREG; 
    cli();
    m = millis_counter;
    SREG = sreg;
    return m;
}
void update_dt(struct time_differencePID *time){
    time->last_time = time->time_now;
    time->time_now = millis_();
    time->dt = (time->time_now-time->last_time);
}
void init_dt(struct time_differencePID *time){
    time->time_now  = millis_();
    time->last_time = time->time_now;  // prevents large dt spike on first PID call
    time->dt        = 1;               // safe default (1 ms)
}
void delay_ms(uint32_t ms) {
    uint32_t start = millis_();
    while ((millis_() - start) < ms);
}