/*
 * rf_app.h
 *
 *  Created on: 25 Apr 2019
 *      Author: jdelsol
 */

#ifndef SRC_RF_RF_APP_H_
#define SRC_RF_RF_APP_H_

#include "mbed.h"
#include "nordic-nrf24l01/nrf24l01/nrf24l01.h"
#include "radio_utils.h"
typedef enum {
	WAIT_EVENT 			= 0,
	EVENT_DETECTED,
	PROCESS_RX,
	PROCESS_TX,
	SWITCH_MODE,
	TRANSFER
}rf_com_state_t;

class RF_app
{

public:
	enum class RFAppMode : uint8_t {
		TX		= 0,
		RX		= 1
	};

	enum class RFAppInterrupt : uint8_t {
		on_RX	= 0,
		on_TX	= 1,
		None	= 2
	};

	RF_app(NRF24L01 *device, RF_app::RFAppMode rf_mode = RFAppMode::TX,
			RF_app::RFAppInterrupt interrupt_enable = RFAppInterrupt::on_TX,
			uint16_t frequency = 2402, uint8_t *Tx_addr = NULL, uint8_t packet_size = 32);

	RF_app(NRF24L01 *device, RF_app::RFAppMode rf_mode = RFAppMode::TX,
			uint16_t frequency = 2402, uint8_t *Tx_addr = NULL, uint8_t packet_size = 32);

	//virtual ~RF_app();

	void setup(RFAppMode rf_mode, RFAppInterrupt enable_interrupt, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size);

	void setup(RFAppMode rf_mode, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size);

	void run(void);

	void exit(void);

	void print_setup(int arg_cnt, char **args);

	void rf_set_config(int arg_cnt, char **args);

	void set_rf_param(int arg_cnt, char **args);

protected:
	void _process(void);

	void _rf_callback(void);

private:
	//static SingletonPtr<PlatformMutex> _mutex;
	NRF24L01 *_device;
	com_packet_t _packet;
	Thread *_thread;
	rf_com_state_t _rf_state;

};

void print_frame(com_packet_t *packet_to_print);


#endif /* SRC_RF_RF_APP_H_ */
