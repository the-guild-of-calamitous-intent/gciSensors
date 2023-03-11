/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#ifndef IMU_USE_UNCALIBRATED_DATA
  #define IMU_USE_UNCALIBRATED_DATA 1
#endif

// sensor drivers
#include "bmp390/bmp3.hpp"
#include "lis3mdl/lis3mdl.hpp"
#include "lsm6dsox/lsm6dsox.hpp"

// range driver
#include "tfmini/tfmini.h"

// filters
#include "filters.hpp"

// helpers
#include "earth.hpp"
// #include "timers.hpp"
#include "units.hpp"