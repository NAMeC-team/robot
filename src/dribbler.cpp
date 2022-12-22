/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dribbler.h"

static Mutex spi_mutex;

Dribbler::Dribbler(SPI *spi, PinName chip_select):
        _spi(spi),
        _chip_select(chip_select, 1),
        _current_command(Commands_STOP),
        _current_speed(0)
{
}

Dribbler::Dribbler_error Dribbler::send_message()
{
    MainBoardToDribbler message = MainBoardToDribbler_init_zero;
    memset(_dribbler_tx_buffer, 0, sizeof(_dribbler_tx_buffer));

    /* Create a stream that will write to our buffer. */
    pb_ostream_t tx_stream
            = pb_ostream_from_buffer(_dribbler_tx_buffer + 1, MainBoardToDribbler_size);

    message.command = _current_command;
    message.speed = _current_speed;

    /* Now we are ready to encode the message! */
    bool status = pb_encode(&tx_stream, MainBoardToDribbler_fields, &message);
    size_t message_length = tx_stream.bytes_written;

    /* Then just check for any errors.. */
    if (!status) {
        printf("[DRIBBLER] Encoding failed: %s\n", PB_GET_ERROR(&tx_stream));
        return Dribbler_error::ENCODE_ERROR;
    }

    /* add message length */
    _dribbler_tx_buffer[0] = message_length;

    /* Send the SPI message */
    spi_mutex.lock();
    _chip_select = 0;
    _spi->write((const char *)_dribbler_tx_buffer,
            MainBoardToDribbler_size + 1, nullptr, 0);
    _chip_select = 1;
    spi_mutex.unlock();

    return Dribbler_error::NO_ERROR;
}

void Dribbler::set_state(Commands state)
{
    _current_command = state;
    send_message();
}

void Dribbler::set_speed(float speed)
{
    _current_speed = speed;
    send_message();
}