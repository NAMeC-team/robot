/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "brushless_board.h"

Brushless_board::Brushless_board(SPI *spi, PinName chip_select):
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

    ct.compute((void *)_brushless_tx_buffer + 4, message_length, (uint32_t *) _brushless_tx_buffer);
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
    ct.compute((void *)_brushless_rx_buffer + 4, BrushlessToMainBoard_size, &response_crc);
    if (response_crc == *((uint32_t *) _brushless_rx_buffer)) {
        printf("CRC OK\n");
    }
    else {
        printf("CRC_ERROR\n");
        _rx_error_count++;
        return Brushless_error::DECODE_ERROR;
    }

    /* Print the data contained in the message. */
    _tx_error_count = response.error_count;

    return Brushless_error::NO_ERROR;
}
