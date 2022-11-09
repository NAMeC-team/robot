/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "brushless_board.h"

namespace {
#define HALF_PERIOD 500ms
}

/* Peripherals */
static DigitalOut led1(LED1);
static SPI driver_spi(SPI_MOSI_DRV, SPI_MISO_DRV, SPI_SCK_DRV);
static Brushless_board motor1(&driver_spi, SPI_CS_DRV1);
static Brushless_board motor2(&driver_spi, SPI_CS_DRV2);
static Brushless_board motor3(&driver_spi, SPI_CS_DRV3);
static Brushless_board motor4(&driver_spi, SPI_CS_DRV4);

/* Struct definitions */
typedef struct _Motor_speed { 
    float speed1;
    float speed2;
    float speed3;
    float speed4;
} Motor_speed;

void compute_motor_speed(Motor_speed *motor_speed, float normal_speed, float tangential_speed, float angular_speed)
{
    
}


int main()
{
    while (true) {
        led1 = !led1;
        if (led1) {
            printf("Alive!\n");
        }
        ThisThread::sleep_for(HALF_PERIOD);
    }
}
