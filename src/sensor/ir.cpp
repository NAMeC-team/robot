#include "ir.h"

#define IR_THRESHOLD 0.6

namespace ir {
    float value = 0.0;
    DigitalOut emit(IR_EMIT, 0.0);
    AnalogIn receive(IR_RECEIVE);

    void compute() {
        emit = 1;
        value = receive.read();
        emit = 0;
    }

    bool present() { return value < IR_THRESHOLD ? true : false; }
}  // namespace ir