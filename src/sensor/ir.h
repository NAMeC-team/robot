#pragma once

#include <mbed.h>

namespace ir {
    extern float value;
    void compute();
    bool present();
}  // namespace ir