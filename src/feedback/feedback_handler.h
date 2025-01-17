#ifndef ROBOT_FEEDBACK_HANDLER_H
#define ROBOT_FEEDBACK_HANDLER_H

#include <mbed.h>
#include <nrf24l01.h>
#include <radio_feedback.pb.h>

class FeedbackHandler {

    NRF24L01& _radio;
    /**
     * Try-locked to check whether the nRF radio is currently
     * sending a response ACK.
     * It's here to ensure that we don't try to write a new payload
     * for the next ACK while the radio is already responding with an ACK.
     * TODO: might not be required
     */
    Mutex& _radio_mutex;

    /**
     * used to periodically update
     * the TX FIFO of the nRF module
     * used to transmit the payload ACK.
     */
    EventQueue& _event_queue;

    /**
     * current radio feedback
     */
    RadioFeedback _feedback;

    /**
     * Pipe for which the ACK payload packet is loaded
     */
    NRF24L01::RxAddressPipe _pipe;

    /**
     * Builds a new feedback packet,
     * and stores it in the object.
     * If the nRF can be accessed (try-lock on mutex),
     * writes into the TX FIFO the feedback packet
     * to be used as payload data for next ACK.
     */
    void write_payload_ack();

public:
    FeedbackHandler(NRF24L01& dev, Mutex& radio_mutex, EventQueue& eq, NRF24L01::RxAddressPipe pipe);

    /**
     * Registers the required call to the EventQueue,
     * effectively starting computation of feedbacks
     */
    void start();
};

#endif // ROBOT_FEEDBACK_HANDLER_H
