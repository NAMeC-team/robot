#pragma once

#include <mbed.h>

namespace ir {
    extern float value;
    void setup();
    void change_emit();
    void compute();
    bool present();
}  // namespace ir