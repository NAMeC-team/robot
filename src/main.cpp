#include <mbed.h>
#include <radio_command.pb.h>
#include <radio_feedback.pb.h>
#include <base_wrapper.pb.h>
#include <swo.h>
#include <rf_app.h>

#include "motor/brushless_board.h"
#include "motor/dribbler.h"
#include "sensor/ir.h"
#include "ssl-kicker.h"

#define HALF_PERIOD 500ms
#define ROBOT_RADIUS 0.085
#define MAX_DRIBBLER_SPEED 400.

// Radio frequency
#define RF_FREQUENCY_1 2508
#define RF_FREQUENCY_2 2510
using namespace sixtron;

static SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
}

DigitalOut led(LED1);

EventQueue event_queue;

static SPI driver_spi(SPI_MOSI_DRV, SPI_MISO_DRV, SPI_SCK_DRV);
static Mutex spi_mutex;
static Dribbler dribbler(&driver_spi, SPI_CS_DRV5, &spi_mutex);

static UnbufferedSerial serial_port(USBTX, USBRX);


void set_dribbler_speed(float dribbler_speed) {
    if (dribbler_speed > 0.0) {
        dribbler.set_state(Commands_RUN);
        if (dribbler_speed > MAX_DRIBBLER_SPEED)
            dribbler_speed = MAX_DRIBBLER_SPEED;
        dribbler.set_speed(dribbler_speed);
        dribbler.send_message();
    } else {
        dribbler.set_state(Commands_STOP);
        dribbler.set_speed(0);
        dribbler.send_message();
    }
}


void on_serial_rx_interrupt() {
    static bool start_of_frame = false;
    static uint8_t length = 0;
    static uint8_t read_count = 0;
    static uint8_t read_buffer[PCToBase_size];
    uint8_t c;

    if (!start_of_frame) {
        //TODO: if we exceed 6 robots, the size of the packet exceeds 8 bits,
        // thus the length will be stored on 2 bytes, making this algorithm invalid
        // not a problem for DivB, but it will be a problem for DivA
        serial_port.read(&c, 1);
        if (c > 0 && c <= (PCToBase_size)) { // Get packet length
            start_of_frame = true;
            length = c;
            read_count = 0;
            // event_queue.call(printf, "Receiving : %d\n", length);
        } else if (c == 0) { // When length is 0 it is the default protobuf packet
            start_of_frame = false;
            length = 0;
            read_count = 0;
            // TODO: Make something
        }
    } else {
        serial_port.read(&read_buffer[read_count], 1);
        read_count++;
        if (read_count == length) {
            read_count = 0;
            start_of_frame = false;
            pb_istream_t rx_stream = pb_istream_from_buffer(read_buffer, length);
            PCToBase message = PCToBase_init_zero;
            bool status = pb_decode(&rx_stream, PCToBase_fields, &message);
            if (!status) {
                return;
            }
            if (message.commands_count <= 0) {
                return;
            }
            uint8_t dribbler_speed = message.commands[0].dribbler;
            event_queue.call(set_dribbler_speed, dribbler_speed);
            led = true;
        }
    }
}

int main()
{
    driver_spi.frequency(500000);

    // Remote
    serial_port.baud(115200);
    serial_port.attach(&on_serial_rx_interrupt, SerialBase::RxIrq);

    event_queue.dispatch_forever();

}
