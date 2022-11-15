/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "brushless_board.h"

Brushless_board::Brushless_board(SPI *spi, PinName chip_select):
        _communication_thread(),
        _event_queue(),
        _communication_id(0),
        _period_ms(100),
        _spi(spi),
        _chip_select(chip_select),
        _tx_error_count(0),
        _rx_error_count(0),
        _current_command(Commands_STOP),
        _current_speed(0)
{
}

Brushless_board::Brushless_error Brushless_board::send_message()
{
    MainBoardToBrushless message = MainBoardToBrushless_init_zero;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t tx_stream
            = pb_ostream_from_buffer(_brushless_tx_buffer + 4, sizeof(MainBoardToBrushless_size));

    message.command = _current_command;
    message.speed = _current_speed;

    /* Now we are ready to encode the message! */
    bool status = pb_encode(&tx_stream, MainBoardToBrushless_fields, &message);
    size_t message_length = tx_stream.bytes_written;

    /* Then just check for any errors.. */
    if (!status) {
        printf("Encoding failed: %s\n", PB_GET_ERROR(&tx_stream));
        return Brushless_error::ENCODE_ERROR;
    }

    /* Compute CRC */
    MbedCRC<POLY_32BIT_ANSI, 32> ct;

    ct.compute(
            (uint8_t *)_brushless_tx_buffer + 4, message_length, (uint32_t *)_brushless_tx_buffer);
    // printf("The CRC of protobuf packet is : 0x%lx\n", _brushless_tx_buffer);

    /* Send the SPI message */
    _chip_select = 0;
    _spi->write((const char *)_brushless_tx_buffer,
            message_length + 4,
            (char *)_brushless_rx_buffer,
            BrushlessToMainBoard_size + 4);
    _chip_select = 1;

    /* Try to decode protobuf response */
    BrushlessToMainBoard response = BrushlessToMainBoard_init_zero;

    /* Create a stream that reads from the buffer. */
    pb_istream_t rx_stream
            = pb_istream_from_buffer(_brushless_rx_buffer + 4, BrushlessToMainBoard_size);

    /* Now we are ready to decode the message. */
    status = pb_decode(&rx_stream, BrushlessToMainBoard_fields, &response);
    printf("[DEBUG]: rx_stream.bytes_left : %d\n", rx_stream.bytes_left);

    /* Check for errors... */
    if (!status) {
        printf("Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
        return Brushless_error::DECODE_ERROR;
    }

    /* Check response CRC */
    uint32_t response_crc = 0;
    ct.compute((uint8_t *)_brushless_rx_buffer + 4, BrushlessToMainBoard_size, &response_crc);
    if (response_crc == *((uint32_t *)_brushless_rx_buffer)) {
        printf("CRC OK\n");
    } else {
        printf("CRC_ERROR\n");
        _rx_error_count++;
        return Brushless_error::DECODE_ERROR;
    }

    /* Print the data contained in the message. */
    _tx_error_count = response.error_count;

    return Brushless_error::NO_ERROR;
}

void Brushless_board::start_communication()
{
    _communication_id = _event_queue.call_every(_period_ms, this, &Brushless_board::send_message);
    _communication_thread.start(callback(&_event_queue, &EventQueue::dispatch_forever));
}

void Brushless_board::stop_communication()
{
    _event_queue.cancel(_communication_id);
}

void Brushless_board::set_communication_period(uint64_t period_ms)
{
    _period_ms = period_ms;
}

void Brushless_board::set_state(Commands state)
{
    _current_command = state;
}

void Brushless_board::set_speed(float speed)
{
    _current_speed = speed;
}

uint32_t Brushless_board::get_rx_error_count()
{
    return _rx_error_count;
}

uint32_t Brushless_board::get_tx_error_count()
{
    return _tx_error_count;
}