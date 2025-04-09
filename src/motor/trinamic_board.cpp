#include <mbed.h>

#include "trinamic_board.h"
static DigitalOut chip_select[5]
        = { SPI_CS_DRV1, SPI_CS_DRV2, SPI_CS_DRV3, SPI_CS_DRV4, SPI_CS_DRV5 };

static SPI *driver_spi = NULL;

// icID is used to identify the chip to be selected
void tmc4671_readWriteSPI(uint16_t icID, uint8_t *data, size_t data_length)
{
    // the _peripheral attribute is a spi_peripheral_s type, and correspond to the slave

    // acquire exclusive access to this SPI bus
    driver_spi->lock();

    // print_hexa_format(data, "Command: ", data_length);

    // operator = has been redefined
    // is it related to the pin we want, or the selection of the chip (slave select ?)
    chip_select[icID] = 0;

    int nb_bits_used = 0;

    // data[0] represents the write_bit
    nb_bits_used = driver_spi->write((char *)data, data_length, (char *)data, data_length);

    // print_hexa_format(data, "Response: ", data_length);

    // we can check if nb_bits_used == data_length

    chip_select[icID] = 1;

    // release exclusive access to this SPI bus
    driver_spi->unlock();
}

TrinamicMotor::TrinamicMotor()
{
}

TrinamicMotor::~TrinamicMotor()
{
}

UnbufferedSerial serial_log(USBTX, USBRX);
#define BUFSIZE 200
#define print_value(format, ...)                                                                   \
    {                                                                                              \
        char buffer[BUFSIZE];                                                                      \
        memset(buffer, 0, BUFSIZE);                                                                \
        int l = sprintf(buffer, format, ##__VA_ARGS__);                                            \
        serial_log.write(buffer, l);                                                               \
    }

void TrinamicMotor::setup(SPI *spi)
{
    for (int i = 0; i < 5; ++i) {
        chip_select[i] = 1;
    }

    if (driver_spi == NULL)
        driver_spi = spi;

    serial_log.baud(115200);
    // Motor type &  PWM configuration
    tmc4671_writeRegister(0, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
    wait_us(2000000);
    print_value(
            "MEOW %d\n", tmc4671_readRegister(0, TMC4671_MOTOR_TYPE_N_POLE_PAIRS) == 0x00030008);
    tmc4671_writeRegister(0, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeRegister(0, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeRegister(0, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeRegister(0, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeRegister(0, TMC4671_ADC_I_SELECT, 0x24000100);
    tmc4671_writeRegister(0, TMC4671_DSADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeRegister(0, TMC4671_DSADC_MCLK_A, 0x20000000);
    tmc4671_writeRegister(0, TMC4671_DSADC_MCLK_B, 0x20000000);
    tmc4671_writeRegister(0, TMC4671_DSADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeRegister(0, TMC4671_ADC_I0_SCALE_OFFSET, 0x00328394);
    tmc4671_writeRegister(0, TMC4671_ADC_I1_SCALE_OFFSET, 0x003283FE);

    // Open loop settings
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_MODE, 0x00000000);
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000000A);

    // Feedback selection
    tmc4671_writeRegister(0, TMC4671_PHI_E_SELECTION, 0x00000002);
    tmc4671_writeRegister(0, TMC4671_UQ_UD_EXT, 0x00000913);

    // ===== Open loop test drive =====

    // Switch to open loop velocity mode
    tmc4671_writeRegister(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

    // Rotate right
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000003C);
    wait_us(2000);

    // Rotate left
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFC4);
    wait_us(4000);

    // Stop
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
    wait_us(2000);
    tmc4671_writeRegister(0, TMC4671_UQ_UD_EXT, 0x00000000);
}

void TrinamicMotor::set_speed(int32_t speed)
{
    tmc4671_writeRegister(0, TMC4671_UQ_UD_EXT, 0x00000913);
    tmc4671_writeRegister(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000113C);
    wait_us(2000000);
    tmc4671_writeRegister(0, TMC4671_UQ_UD_EXT, 0x00000000);
}