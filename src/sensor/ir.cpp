#include "ir.h"

#define IR_THRESHOLD 0.6

namespace ir {
float value = 0.0;
DigitalOut emit(IR_EMIT, 0.0);
AnalogIn receive(IR_RECEIVE);
uint8_t step = 0;

// 200 us, 75 %
Ticker pwm_ticker;
uint8_t state;

UnbufferedSerial serial_port(USBTX, USBRX); // probably 9600 baudrate, I kinda forgot

void setup()
{
    pwm_ticker.attach_us(&ir::change_emit, 50);
}

void change_emit()
{
    step += 1;
    if (step >= 3) {
        emit = !emit;
    } else if (step >= 4) {
        emit = !emit;
        step = 0;
    }
}

void compute()
{
    value = receive.read();
    char buffer[32];
    // sprintf(buffer, "IR: %s\n", value ? "ON" : "off");
    sprintf(buffer, "IR: %f\n", value); // use this variant to print the IR value perceived instead
    serial_port.write(buffer, strlen(buffer));
}

bool present()
{
    return value < IR_THRESHOLD ? true : false;
}
} // namespace ir