#include <mbed.h>
#include <radio_command.pb.h>
#include <radio_feedback.pb.h>
#include <swo.h>

#include "motor/brushless_board.h"
#include "motor/dribbler.h"
#include "rf_app.h"
#include "sensor/ir.h"
#include "ssl-kicker.h"

#define HALF_PERIOD 500ms
#define ROBOT_RADIUS 0.085

// Radio frequency
#define RF_FREQUENCY_1 2402
#define RF_FREQUENCY_2 2460
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
static Brushless_board motor1(&driver_spi, SPI_CS_DRV1, &spi_mutex);
static Brushless_board motor2(&driver_spi, SPI_CS_DRV2, &spi_mutex);
static Brushless_board motor3(&driver_spi, SPI_CS_DRV3, &spi_mutex);
static Brushless_board motor4(&driver_spi, SPI_CS_DRV4, &spi_mutex);
static Dribbler dribbler(&driver_spi, SPI_CS_DRV5, &spi_mutex);
static SPI radio_spi(SPI_MOSI_RF, SPI_MISO_RF, SPI_SCK_RF);
static NRF24L01 radio1(&radio_spi, SPI_CS_RF1, CE_RF1, IRQ_RF1);
static NRF24L01 radio2(&radio_spi, SPI_CS_RF2, CE_RF2, IRQ_RF2);

static Timeout timeout;

static KICKER kicker(KCK_EN, KCK1, KCK2);

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

    event_queue.call_every(16ms, ir::compute);

    event_queue.dispatch_forever();

    while (true) { }
}
