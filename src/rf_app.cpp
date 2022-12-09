/*
 * rf_app.cpp
 *
 *  Created on: 25 Apr 2019
 *      Author: jdelsol
 */
#include "mbed.h"
#include "rf_app.h"

namespace {
#define RF_USAGE_DISPLAY		"\r\nusage: RFx <option> -<param> <value>"
#define RF_OPTION_DISPLAY		"\r\n<option>: "\
								"\r\n\t set: set a value of rf device parameter"\
								"\r\n\t get: get a value of rf device parameter"\
								"\r\n\t start: restart module"\
								"\r\n\t stop: stop the rf process"\
								"\r\n\t -h: show RF help display"
#define RF_PARAM_DISPLAY		"\r\n<param>:"\
								"\r\n\t -frequency: frequency in Hz of rf device <2400-2525>"\
								"\r\n\t -mode: set mode Rx/Tx <0: Tx; 1:Rx>"\
								"\r\n\t -addr: address in hexa to attach RF rx payload into pipe0 <0x1122334455>"\
								"\r\n\t -txaddr: (only used in Tx mode) address in hexa of tx device will transmit the data<0x1122334455>"\
								"\r\n\t -interrupt: enable/disable interrupt <0:none; 1:rx_only; 2:tx_only>"
#define RF_FREQ_MAX 2525
#define RF_FREQ_MIN 2400
}


typedef enum {
	RX_EVENT	= 1,
	TX_EVENT	= 2,
}rf_com_events_t;

typedef struct {
	uint8_t id_app;
	bool print_process;
	bool process_running;
}rf_app_param_t;

static rf_com_events_t rf_event; // not used
static rf_app_param_t rf_app_param;

RF_app::RF_app(NRF24L01 *device, RFAppMode rf_mode, RFAppInterrupt enable_interrupt, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size):
		_device(device),
		_packet(),
		_thread()
{
	_rf_state = WAIT_EVENT;
	setup(rf_mode, enable_interrupt, frequency, Tx_addr, packet_size);
	rf_app_param.id_app++;
}

RF_app::RF_app(NRF24L01 *device, RFAppMode rf_mode, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size):
		_device(device),
		_packet(),
		_thread()
{
	_rf_state = WAIT_EVENT;
	setup(rf_mode, frequency, Tx_addr, packet_size);
	rf_app_param.id_app++;
}

void RF_app::setup(RFAppMode rf_mode, RFAppInterrupt enable_interrupt, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size)
{
	switch (rf_mode) {
		case RFAppMode::RX:

			_device->initialize(NRF24L01::OperationMode::RECEIVER, NRF24L01::DataRate::_2MBPS, frequency);
			_device->attach_receive_payload(NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
			if (enable_interrupt == RFAppInterrupt::on_RX) {
				_device->set_interrupt(NRF24L01::InterruptMode::RX_ONLY);
				_device->attach(callback(this, &RF_app::_rf_callback));
			} else {
				_device->set_interrupt(NRF24L01::InterruptMode::NONE);
			}
			_device->start_listening();
			_rf_state = WAIT_EVENT;
			break;

		case RFAppMode::TX:
			_device->initialize(NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, frequency);
			_device->attach_transmitting_payload(NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
			if (enable_interrupt == RFAppInterrupt::on_TX) {
				_device->set_interrupt(NRF24L01::InterruptMode::TX_ONLY);
				_device->attach(callback(this, &RF_app::_rf_callback));
			} else {
				_device->set_interrupt(NRF24L01::InterruptMode::NONE);
			}
			_rf_state = PROCESS_TX;
			break;
	}
}

void RF_app::setup(RFAppMode rf_mode, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size)
{
	switch (rf_mode) {
		case RFAppMode::RX:
			_device->initialize(NRF24L01::OperationMode::RECEIVER, NRF24L01::DataRate::_2MBPS, frequency);
			_device->attach_receive_payload(NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
			_device->set_interrupt(NRF24L01::InterruptMode::RX_ONLY);
			_device->attach(callback(this, &RF_app::_rf_callback));
			_device->start_listening();
			_rf_state = WAIT_EVENT;
			break;

		case RFAppMode::TX:
			_device->initialize(NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, frequency);
			_device->attach_transmitting_payload(NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
			_device->set_interrupt(NRF24L01::InterruptMode::TX_ONLY);
			_device->attach(callback(this, &RF_app::_rf_callback));
			_rf_state = PROCESS_TX;
			break;
	}
}

void RF_app::run()
{
	_thread = new Thread();
	_thread->start(callback(this, &RF_app::_process));
	rf_app_param.process_running = true;
}

void RF_app::exit()
{
	_thread->terminate();
	delete _thread;
	rf_app_param.process_running = false;
}

void RF_app::_process(void)
{
	rf_app_param.print_process = false;

	while (true) {
		switch (this->_rf_state) {
			case WAIT_EVENT:
				if (_device->mode() == NRF24L01::OperationMode::RECEIVER) {
					// wait events
					ThisThread::sleep_for(5ms);
				} else {
					ThisThread::sleep_for(1s);
					_rf_state = PROCESS_TX;
				}
				break;

			case EVENT_DETECTED:
				if (_device->mode() == NRF24L01::OperationMode::RECEIVER) {
					this->_rf_state = PROCESS_RX;
				} else {
					this->_rf_state = PROCESS_TX;
				}
				break;
			case PROCESS_RX:
				if (_device->mode() == NRF24L01::OperationMode::RECEIVER) {
					_device->clear_interrupt_flags();
					if (((_device->fifo_status_register()) & 0x02) == 0x02) {
						// rx fifo full
						_device->read_packet(_packet.raw, _device->payload_size());
						// TODO: parse data
						frame_parser(&_packet);
						if (rf_app_param.print_process == true) {
							print_frame(&_packet);
						}
						// TODO: exec frame
				}
				//back to wait state
				this->_rf_state = WAIT_EVENT;
				break;
			case PROCESS_TX:
				//back to listenning state
				// create_frame(PACKET_ROBOT, &_packet);
				if (rf_app_param.print_process == true) {
					print_frame(&_packet);
				}
				_device->send_packet(_packet.raw, _packet.total_length);
				_rf_state = WAIT_EVENT;
				break;
			case SWITCH_MODE:
				break;
			case TRANSFER:
				break;
			}
		}
	}
}

void RF_app::print_setup(int arg_cnt, char **args)
{
	size_t i;
	uint8_t rx_addr_device[5] = {0};
	uint8_t tx_addr_device[5] = {0};

	// printf config RF 1
	printf("\r\n\r\n**** RF ****");
	if (_device->mode() == NRF24L01::OperationMode::RECEIVER) {
		printf("\r\nMode : RECEIVER");
	} else {
		printf("\r\nMode : TRANSCEIVER");
	}
	printf("\r\nFrequency : %d Hz", _device->rf_frequency());
	printf("\r\nOutput power : %d dBm", (int)_device->rf_output_power());
	printf("\r\nData rate : %d Kbps", (int)_device->data_rate());
	_device->rx_address(NRF24L01::RxAddressPipe::RX_ADDR_P0, rx_addr_device);
	printf("\r\nRx Address 0x");
	for (i = 0; i < sizeof(rx_addr_device); i++) {
		printf("%.2X", rx_addr_device[i]);
	}
	_device->tx_address(tx_addr_device);
	printf("\r\nTx Address 0x");
	for (i = 0; i < sizeof(tx_addr_device); i++) {
		printf("%.2X", tx_addr_device[i]);
	}
}

void RF_app::set_rf_param(int arg_cnt, char **args)
{
	if (rf_app_param.print_process != true) {
		rf_app_param.print_process = true;
		printf("\r\nlog RF enable");
	} else {
		rf_app_param.print_process = false;
		printf("\r\nlog RF disable");
	}
}

//void RF_app::switch_mode(RF_app::RFAppMode next_mode)
//{
//	switch (next_mode) {
//		case RF_app::RFAppMode::RX:
//			break;
//		case RF_app::RFAppMode::TX:
//					break;
//	}
//}


/*__________________________

 	 Callback
___________________________ */
void RF_app::_rf_callback(void)
{
	this->_rf_state = EVENT_DETECTED;
}

/*__________________________

 	 Extern C functions
___________________________ */
void print_frame(com_packet_t *packet_to_print)
{
	printf("\r\n");
	switch (packet_to_print->type) {
		case PACKET_MASTER:
			printf("Type : MASTER\r\n");
			break;
		case PACKET_PARAMS:
			printf("Type : PARAMS\r\n");
			break;
		case PACKET_ROBOT:
			printf("Type : ROBOT\r\n");
			break;
	}

	// printf total length
	printf("length : %d bytes\r\n", packet_to_print->total_length);

	switch (packet_to_print->type) {
		case PACKET_MASTER:
			printf("actions : 0x%.2X\r\n", packet_to_print->master.actions);
			printf("x_speed : 0x%.2X\r\n", packet_to_print->master.x_speed);
			printf("y_speed : 0x%.2X\r\n", packet_to_print->master.y_speed);
			printf("t_speed : 0x%.2X\r\n", packet_to_print->master.t_speed);
			printf("kickPower : 0x%.2X\r\n", packet_to_print->master.kickPower);
			break;
		case PACKET_PARAMS:
			printf("kp : %f\r\n", packet_to_print->params.kp);
			printf("ki : %f\r\n", packet_to_print->params.ki);
			printf("kd : %f\r\n", packet_to_print->params.kd);
			break;
		case PACKET_ROBOT:
			printf("id : 0x%.2X\r\n", packet_to_print->robot.id);
			printf("status : 0x%.2X\r\n", packet_to_print->robot.status);
			printf("cap_volt : 0x%.2X\r\n", packet_to_print->robot.cap_volt);
			printf("voltage : 0x%.2X\r\n", packet_to_print->robot.voltage);
			break;
	}
}


