/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BRUSHLESS_BOARD_H_
#define BRUSHLESS_BOARD_H_

#include "brushless.pb.h"
#include "mbed.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

class Brushless_board {
public:
    enum class Brushless_error : int8_t {
        NO_ERROR = 0,
        SPI_ERROR = -1,
        ENCODE_ERROR = -2,
        DECODE_ERROR = -3
    };

    enum class Motor_state : uint8_t {
        RUN = 0,
        STOP = 1,
        BREAK = 2
    };

    Brushless_board(SPI *spi, PinName chip_select, Mutex *spi_mutex);

    /**
     * @brief Start the communication thread that send messages to the board periodically
     * 10 Hz by default
     *
     */
    void start_communication();

    void stop_communication();

    void set_communication_period(uint64_t period_ms);

    void set_speed(float speed);

    void set_state(Commands state);

    uint32_t get_tx_error_count();

    uint32_t get_rx_error_count();

    Brushless_error send_message();

private:
    // RTOS
    Thread _communication_thread;
    EventQueue _event_queue;
    int _communication_id;
    uint64_t _period_ms;

    // SPI
    SPI *_spi;
    DigitalOut _chip_select;
    Mutex *_spi_mutex;

    // Protobuf data
    uint32_t _tx_error_count;
    uint32_t _rx_error_count;
    Commands _current_command;
    float _current_speed;

    uint8_t _brushless_tx_buffer[MainBoardToBrushless_size + 4 + 1]; // CRC size is 4 bytes
    uint8_t _brushless_rx_buffer[BrushlessToMainBoard_size + 4 + 1]; // CRC size is 4 bytes + length
};

#endif // BRUSHLESS_BOARD_H_
