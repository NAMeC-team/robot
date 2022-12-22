/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DRIBBLER_H_
#define DRIBBLER_H_

#include "mbed.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "ssl_data.pb.h"

class Dribbler {
public:
    enum class Dribbler_error : int8_t {
        NO_ERROR = 0,
        SPI_ERROR = -1,
        ENCODE_ERROR = -2
    };

    enum class Motor_state : uint8_t {
        RUN = 0,
        STOP = 1,
        BREAK = 2
    };

    Dribbler(SPI *spi, PinName chip_select);

    void set_speed(float speed);

    void set_state(Commands state);

private:
    // SPI
    SPI *_spi;
    DigitalOut _chip_select;

    // Protobuf data
    Commands _current_command;
    float _current_speed;

    uint8_t _dribbler_tx_buffer[MainBoardToDribbler_size + 4 + 1]; // CRC size is 4 bytes

    Dribbler_error send_message();
};

#endif // DRIBBLER_H_
