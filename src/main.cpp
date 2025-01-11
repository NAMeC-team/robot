#include <mbed.h>
#include <radio_command.pb.h>
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
#define RF_FREQUENCY_1 2509
#define RF_FREQUENCY_2 2511
using namespace sixtron;

static SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
}

EventQueue event_queue;

// SPI handler
static SPI driver_spi(SPI_MOSI_DRV, SPI_MISO_DRV, SPI_SCK_DRV);
static Mutex spi_mutex;

// motors of the robot
static Brushless_board motor1(&driver_spi, SPI_CS_DRV1, &spi_mutex);
static Brushless_board motor2(&driver_spi, SPI_CS_DRV2, &spi_mutex);
static Brushless_board motor3(&driver_spi, SPI_CS_DRV3, &spi_mutex);
static Brushless_board motor4(&driver_spi, SPI_CS_DRV4, &spi_mutex);
static Dribbler dribbler(&driver_spi, SPI_CS_DRV5, &spi_mutex);
static SPI radio_spi(SPI_MOSI_RF, SPI_MISO_RF, SPI_SCK_RF);
static NRF24L01 radio1(&radio_spi, SPI_CS_RF1, CE_RF1, IRQ_RF1);
static Timeout timeout; // used to stop motors when no command received after a certain time

static KICKER kicker(KCK_EN, KCK1, KCK2);

/* Struct definitions */
typedef struct _Motor_speed {
    float speed1;
    float speed2;
    float speed3;
    float speed4;
} Motor_speed;

// RX address at which packets are received from the mainboard
static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };
// Current AI command
RadioCommand ai_message = RadioCommand_init_zero;

/**
 * Compute motors speeds required to be sent to motors
 * @param motor_speed Resulting motor speeds
 */
void compute_motor_speed(
        Motor_speed *motor_speed, float normal_speed, float tangential_speed, float angular_speed)
{
    motor_speed->speed1 = -(sqrt(3) / 2.) * normal_speed + (1. / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
    motor_speed->speed2 = -(sqrt(2) / 2.) * normal_speed - (sqrt(2) / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
    motor_speed->speed3 = (sqrt(2) / 2.) * normal_speed - (sqrt(2) / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
    motor_speed->speed4 = (sqrt(3) / 2.) * normal_speed + (1. / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
}

/**
 * Sends speed commands to all motors
 */
void apply_motor_speed()
{
    Motor_speed motor_speed;
    compute_motor_speed(&motor_speed,
            ai_message.normal_velocity,
            ai_message.tangential_velocity,
            ai_message.angular_velocity);

    motor1.set_state(Commands_RUN);
    motor2.set_state(Commands_RUN);
    motor3.set_state(Commands_RUN);
    motor4.set_state(Commands_RUN);
    motor1.set_speed(motor_speed.speed1);
    motor2.set_speed(motor_speed.speed2);
    motor3.set_speed(motor_speed.speed3);
    motor4.set_speed(motor_speed.speed4);
}

/**
 * Sends a stop command to all motors
 */
void stop_motors()
{
    motor1.set_state(Commands_STOP);
    motor2.set_state(Commands_STOP);
    motor3.set_state(Commands_STOP);
    motor4.set_state(Commands_STOP);
}

/**
 * Called whenever we receive a new packet from the
 * base station
 * @param data Data packet
 */
void on_rx_interrupt(uint8_t *data, size_t data_size)
{
    static uint8_t length = 0;
    ai_message = RadioCommand_init_zero;

    length = data[0];
    event_queue.call(printf, "LENGTH: %d\n", length);
    if (length == 0) {
        event_queue.call(stop_motors);
    } else {
        /* Try to decode protobuf response */
        /* Create a stream that reads from the buffer. */
        pb_istream_t rx_stream = pb_istream_from_buffer(&data[1], length);

        /* Now we are ready to decode the message. */
        bool status = pb_decode(&rx_stream, RadioCommand_fields, &ai_message);

        /* Apply command if no error occured */
        if (!status) {
            event_queue.call(printf, "[IA] Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
        } else {
            if (ai_message.robot_id != ROBOT_ID)
                return;
            event_queue.call(apply_motor_speed);
            if (ai_message.kick == Kicker::Kicker_CHIP) {
                kicker.kick1(ai_message.kick_power);
                event_queue.call(printf, "Power %f\n", ai_message.kick_power);
            }
            if (ai_message.kick == Kicker::Kicker_FLAT) {
                kicker.kick2(ai_message.kick_power);
                event_queue.call(printf, "Power %f\n", ai_message.kick_power);
            }
            if (ai_message.dribbler > 0.0) {
                dribbler.set_state(Commands_RUN);
                float dribbler_speed = ai_message.dribbler;
                if (dribbler_speed > MAX_DRIBBLER_SPEED)
                    dribbler_speed = MAX_DRIBBLER_SPEED;
                dribbler.set_speed(dribbler_speed);
                dribbler.send_message();
            } else {
                dribbler.set_state(Commands_STOP);
                dribbler.set_speed(0);
                dribbler.send_message();
            }

            if (ai_message.charge == 1) {
                kicker.enable_charge();
            } else {
                kicker.disable_charge();
            }
        }
    }
    timeout.detach();
    timeout.attach(stop_motors, 500ms);
}

void print_communication_status()
{
    printf("Motor1 TX errors: %ld\n", motor1.get_tx_error_count());
    printf("Motor1 RX errors: %ld\n", motor1.get_rx_error_count());
    printf("Motor2 TX errors: %ld\n", motor2.get_tx_error_count());
    printf("Motor2 RX errors: %ld\n", motor2.get_rx_error_count());
    printf("Motor3 TX errors: %ld\n", motor3.get_tx_error_count());
    printf("Motor3 RX errors: %ld\n", motor3.get_rx_error_count());
    printf("Motor4 TX errors: %ld\n", motor4.get_tx_error_count());
    printf("Motor4 RX errors: %ld\n", motor4.get_rx_error_count());
}

int main()
{
    driver_spi.frequency(500000);

    motor1.set_communication_period(10);
    motor2.set_communication_period(10);
    motor3.set_communication_period(10);
    motor4.set_communication_period(10);

    motor1.start_communication();
    motor2.start_communication();
    motor3.start_communication();
    motor4.start_communication();

    RF_app rf_app1(&radio1,
            RF_app::RFAppMode::RX,
            RF_FREQUENCY_1,
            com_addr1_to_listen,
            RadioCommand_size + 1);
    rf_app1.print_setup();
    rf_app1.attach_rx_callback(&on_rx_interrupt);
    rf_app1.run();

    event_queue.dispatch_forever();

    while (true) { }
}
