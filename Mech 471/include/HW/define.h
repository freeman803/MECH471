#pragma once

#define BIT(a) (1UL << (a))

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
    output = 0,
    input = 1
} IO;

typedef enum {
    A_0 = 0,
    A_1,
    A_2,
    A_3,
    A_4,
    A_5,
} ANALOG_PINS;
