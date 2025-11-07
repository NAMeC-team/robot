#include "ir.h"

#define IR_THRESHOLD 0.6

namespace ir {
    float value = 0.0;
    DigitalOut emit(IR_EMIT, 0.0);
    AnalogIn receive(IR_RECEIVE);

    UnbufferedSerial serial_port(USBTX, USBRX); // probably 9600 baudrate, I kinda forgot

    void compute() {
        emit = 1;
        value = receive.read();
        emit = 0;
        char buffer[32];
        sprintf(buffer, "IR: %s\n", value ? "ON" : "off");
        // sprintf(buffer, "IR: %f\n", value);  // use this variant to print the IR value perceived instead
        serial_port.write(buffer, strlen(buffer));
    }

    bool present() { return value < IR_THRESHOLD ? true : false; }
}  // namespace ir