#pragma once

#include "common.hpp"
#ifdef PICO_BOARD
#include "sensor_pico.hpp"
#elif ARDUINO
#include "sensor_arduino.hpp"
#elif __linux__
#include "sensor_linux.hpp"
#endif