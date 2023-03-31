#pragma once

#include <mbed.h>
#include <nrf24l01.h>
#include <radio_command.pb.h>

class RF_app {
   public:
    enum class RFAppMode : uint8_t { TX = 0, RX = 1 };

    enum class RFAppInterrupt : uint8_t { on_RX = 0, on_TX = 1, None = 2 };

    RF_app(NRF24L01 *device, RF_app::RFAppMode rf_mode = RFAppMode::TX,
           RF_app::RFAppInterrupt interrupt_enable = RFAppInterrupt::on_TX,
           uint16_t frequency = 2402, uint8_t *Tx_addr = NULL,
           uint8_t packet_size = 32);

    RF_app(NRF24L01 *device, RF_app::RFAppMode rf_mode = RFAppMode::TX,
           uint16_t frequency = 2402, uint8_t *Tx_addr = NULL,
           uint8_t packet_size = 32);

    // virtual ~RF_app();

    void setup(RFAppMode rf_mode, RFAppInterrupt enable_interrupt,
               uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size);

    void setup(RFAppMode rf_mode, uint16_t frequency, uint8_t *Tx_addr,
               uint8_t packet_size);

    void run(void);

    void exit(void);

    void print_setup();

    void attach_rx_callback(
        Callback<void(uint8_t *packet, size_t length)> rx_callback);

    void get_rx_packet();

   protected:
    void _process(void);

    void _rf_callback(void);

   private:
    NRF24L01 *_device;
    Thread _thread;
    EventQueue _event_queue;
    Callback<void(uint8_t *packet, size_t length)> _rx_callback;
    uint8_t _packet[RadioCommand_size];
};
