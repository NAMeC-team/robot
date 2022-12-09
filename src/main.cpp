/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "brushless_board.h"
#include "mbed.h"
#include "swo.h"

namespace {
#define HALF_PERIOD 500ms
#define WHEEL_RADIUS 0.03
}

using namespace sixtron;
SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
}

/* Peripherals */
static DigitalOut led1(LED1);
static UnbufferedSerial serial_port(USBTX, USBRX);
static SPI driver_spi(SPI_MOSI_DRV, SPI_MISO_DRV, SPI_SCK_DRV);
static Brushless_board motor1(&driver_spi, SPI_CS_DRV4);
static Brushless_board motor2(&driver_spi, SPI_CS_DRV1);
static Brushless_board motor3(&driver_spi, SPI_CS_DRV2);
static Brushless_board motor4(&driver_spi, SPI_CS_DRV3);
static DigitalOut cs_drv5(SPI_CS_DRV5, 1);

/* RTOS */
EventQueue event_queue;

/* Struct definitions */
typedef struct _Motor_speed {
    float speed1;
    float speed2;
    float speed3;
    float speed4;
} Motor_speed;

IAToMainBoard ai_message = IAToMainBoard_init_zero;

void compute_motor_speed(
        Motor_speed *motor_speed, float normal_speed, float tangential_speed, float angular_speed)
{
    motor_speed->speed1 = -(sqrt(3) / 2.) * normal_speed + (1. / 2.) * tangential_speed
            + WHEEL_RADIUS * angular_speed;
    motor_speed->speed2 = -(sqrt(2) / 2.) * normal_speed - (sqrt(2) / 2.) * tangential_speed
            + WHEEL_RADIUS * angular_speed;
    motor_speed->speed3 = (sqrt(2) / 2.) * normal_speed - (sqrt(2) / 2.) * tangential_speed
            + WHEEL_RADIUS * angular_speed;
    motor_speed->speed4 = (sqrt(3) / 2.) * normal_speed + (1. / 2.) * tangential_speed
            + WHEEL_RADIUS * angular_speed;
}

void apply_motor_speed()
{
    if (ai_message.normal_speed != 0 || ai_message.tangential_speed != 0
            || ai_message.angular_speed != 0) {
        printf("Normal speed: %f\n", ai_message.normal_speed);
        printf("tangential speed: %f\n", ai_message.tangential_speed);
        printf("angular speed: %f\n", ai_message.angular_speed);
    }

    Motor_speed motor_speed;
    compute_motor_speed(&motor_speed,
            ai_message.normal_speed,
            ai_message.tangential_speed,
            ai_message.angular_speed);
    // printf("%f\n", motor_speed.speed1);
    // printf("%f\n", motor_speed.speed2);
    // printf("%f\n", motor_speed.speed3);
    // printf("%f\n", motor_speed.speed4);
    motor1.set_speed(motor_speed.speed1);
    motor2.set_speed(motor_speed.speed2);
    motor3.set_speed(motor_speed.speed3);
    motor4.set_speed(motor_speed.speed4);
}

void on_rx_interrupt()
{
    static bool start_of_frame = false;
    static uint8_t length = 0;
    static uint8_t read_count = 0;
    static uint8_t read_buffer[IAToMainBoard_size];
    uint8_t c;

    if (!start_of_frame) {
        serial_port.read(&c, 1);
        if (c > 0 && c <= (IAToMainBoard_size)) {
            start_of_frame = true;
            length = c;
            read_count = 0;
            // event_queue.call(printf, "Receiving : %d\n", length);
        } else if (c == 0) {
            start_of_frame = false;
            length = 0;
            read_count = 0;
            ai_message = IAToMainBoard_init_zero;
            event_queue.call(apply_motor_speed);
        }
    } else {
        serial_port.read(&read_buffer[read_count], 1);
        read_count++;
        if (read_count == length) {
            read_count = 0;
            start_of_frame = false;

            // event_queue.call(printf, "Parsing !\n");

            /* Try to decode protobuf response */
            ai_message = IAToMainBoard_init_zero;

            /* Create a stream that reads from the buffer. */
            pb_istream_t rx_stream = pb_istream_from_buffer(read_buffer, length);

            /* Now we are ready to decode the message. */
            bool status = pb_decode(&rx_stream, IAToMainBoard_fields, &ai_message);

            /* Check for errors... */
            if (!status) {
                event_queue.call(printf, "Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
            } else {
                event_queue.call(apply_motor_speed);
            }
        }
    }
}

void print_communication_status()
{
    printf("Motor1 TX errors: %d\n", motor1.get_tx_error_count());
    printf("Motor1 RX errors: %d\n", motor1.get_rx_error_count());
    printf("Motor2 TX errors: %d\n", motor2.get_tx_error_count());
    printf("Motor2 RX errors: %d\n", motor2.get_rx_error_count());
    printf("Motor3 TX errors: %d\n", motor3.get_tx_error_count());
    printf("Motor3 RX errors: %d\n", motor3.get_rx_error_count());
    printf("Motor4 TX errors: %d\n", motor4.get_tx_error_count());
    printf("Motor4 RX errors: %d\n", motor4.get_rx_error_count());
}

int main()
{
    driver_spi.frequency(1000000);

    // Remote
    serial_port.baud(115200);
    serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);

    motor1.set_communication_period(10);
    motor2.set_communication_period(10);
    motor3.set_communication_period(10);
    motor4.set_communication_period(10);

    motor1.start_communication();
    motor2.start_communication();
    motor3.start_communication();
    motor4.start_communication();

    event_queue.call_every(1s, print_communication_status);

    event_queue.dispatch_forever();

    while (true) { }
}
