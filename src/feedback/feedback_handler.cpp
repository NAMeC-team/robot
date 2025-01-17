#include "feedback_handler.h"
#include <pb_encode.h>

FeedbackHandler::FeedbackHandler(NRF24L01 &dev, Mutex& radio_mutex, EventQueue &eq, NRF24L01::RxAddressPipe pipe):
    _radio(dev), _radio_mutex(radio_mutex), _event_queue(eq), _pipe(pipe)
{
    _feedback = RadioFeedback_init_zero;
}

void FeedbackHandler::start()
{
    _event_queue.call_every(1ms, this, &FeedbackHandler::write_payload_ack);
}

void FeedbackHandler::write_payload_ack()
{
    // build new feedback packet
    _feedback = RadioFeedback_init_zero; // TODO
    _feedback.ir = 1;
    _feedback.robot_id = 976;

    // boil it down to a Protobuf packet
    uint8_t tx_buffer[RadioFeedback_size + 1];
    pb_ostream_t tx_stream
            = pb_ostream_from_buffer(&tx_buffer[1], RadioFeedback_size);
    bool status = pb_encode(&tx_stream, RadioFeedback_fields, &_feedback);
    if (!status) {
        // encoding error
        printf("[FeedbackHandler] couldn't encode packet: Reason : %s\n", PB_GET_ERROR(&tx_stream));
        return;
    }

    uint8_t data_len = tx_stream.bytes_written;
    tx_buffer[0] = data_len;

    // get exclusive access to radio to write
    // into TX FIFO payload ACK register
    if (_radio_mutex.trylock()) {
        _radio.load_auto_ack_payload(tx_buffer, data_len + 1, _pipe);
        _radio_mutex.unlock();
    }


}