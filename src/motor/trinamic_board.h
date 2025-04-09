#pragma once

#include <mbed.h>

extern "C" {
#include "TMC4671.h"
void tmc4671_readWriteSPI(uint16_t icID, uint8_t *data, size_t data_length);
}

class TrinamicMotor {
private:
public:
    TrinamicMotor();
    ~TrinamicMotor();

    void setup(SPI *spi);

    void set_speed(int32_t speed);
};
