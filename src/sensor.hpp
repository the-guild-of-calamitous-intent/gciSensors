#pragma once

#include "common/quaternions.hpp"
#include "common/vectors.hpp"
#include "common/messages.hpp"
#include "common/compfilter.hpp"
#include "common/algorithms.hpp"
#include "common/kf1d.hpp"

#if defined(PICO_BOARD)
#include "sensor_pico_i2c.hpp"
#include "sensor_pico_spi.hpp"
#elif defined(ARDUINO)
#include "sensor_arduino.hpp"
#elif defined(__linux__)
#include "sensor_linux.hpp"
#endif