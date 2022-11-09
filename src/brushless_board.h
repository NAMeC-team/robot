/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BRUSHLESS_BOARD_H_
#define BRUSHLESS_BOARD_H_

#include "mbed.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "sensordata.pb.h"

class Brushless_board {
public:
    enum class Brushless_error : int8_t {
        NO_ERROR = 0,
        SPI_ERROR = -1,
        ENCODE_ERROR = -2,
        DECODE_ERROR = -3
    };

    Brushless_board(SPI *spi, PinName chip_select);

    void set_speed(float speed);

    void run();

    void stop();

    void motor_break();

    uint32_t get_tx_error_count();

    uint32_t get_rx_error_count();

    Brushless_error send_message();

private:
    // SPI
    SPI *_spi;
    DigitalOut _chip_select;

    // Protobuf data
    uint32_t _tx_error_count;
    uint32_t _rx_error_count;
    Commands _current_command;
    float _current_speed;

    uint8_t _brushless_tx_buffer[MainBoardToBrushless_size + 4]; // CRC size is 4 bytes
    uint8_t _brushless_rx_buffer[BrushlessToMainBoard_size + 4]; // CRC size is 4 bytes
};

#endif // BRUSHLESS_BOARD_H_
