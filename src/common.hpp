#pragma once

#include <cmath>

// returns altitude in meters ... same as below, but more details
// plus, most is computed at compile time, so no performance difference
// float altitude1(const float pressure) {
//   // constexpr float inv_seaLevelhPa = 1.0f / 1013.25f;
//   constexpr float inv_seaLevelPa = 1.0f / 101325.0f;
//   // return 44330.0f * (1.0f - powf((pressure) / (seaLevelhPa*100.0f), 0.1903f));
//   return 44330.0f * (1.0f - powf(pressure * inv_seaLevelPa, 0.1903f));
// }

// Same as mean sea level (MSL) altitude
// Altitude from pressure:
// https://www.mide.com/air-pressure-at-altitude-calculator
//
// Returns meters above sealevel
float pressure_altitude(const float p) {
  // constexpr float Tb = 15; // temperature at sea level [C] - doesn't work
  // constexpr float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... diff exp for pow??
  constexpr float Tb  = 288.15f;           // temperature at sea level [K]
  constexpr float Lb  = -0.0065f;          // lapse rate [K/m]
  constexpr float R   = 8.31446261815324f; // universal gas const [Nm/(mol K)]
  constexpr float M   = 0.0289644f; // molar mass of Earth's air [kg/mol]
  constexpr float g0  = 9.80665f;   // gravitational const [m/s^2]
  constexpr float Pb  = 101325.0f;         // pressure at sea level [Pa]

  constexpr float exp = -R * Lb / (g0 * M);
  constexpr float scale  = Tb / Lb;
  const float inv_Pb = 1.0f / Pb;

  return scale * (powf(p * inv_Pb, exp) - 1.0f);
}

namespace gci {

namespace sensors {


///////////////////////////////////////////////////////////
// Vector (X,Y,Z) Sensors
///////////////////////////////////////////////////////////

// template <typename T>
struct vec_t {
  float x, y, z;
  // bool ok;  // error?

  inline const float magnitude() const { return sqrtf(x * x + y * y + z * z); }

  bool normalize() {
    float n = 1.0f / magnitude();
    if (std::isinf(n)) return false;

    x *= n;
    y *= n;
    z *= n;

    return true;
  }

  float& operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};

// using vecf_t = vec_t<float>;
// using vecd_t = vec_t<double>;
// using veci_t = vec_t<int16_t>;


struct vec_raw_t {
  int16_t x, y, z;
  // bool ok;  // error?

  int16_t operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};


///////////////////////////////////////////////////////////
// Orientation
///////////////////////////////////////////////////////////

struct quat_t {
  float w, x, y, z;
  // bool ok;  // error?

  inline const float magnitude() const { return sqrtf(x * x + y * y + z * z + w * w); }

  bool normalize() {
    float n = 1.0f / magnitude();
    if (std::isinf(n)) return false;

    w *= n;
    x *= n;
    y *= n;
    z *= n;

    return true;
  }

  float& operator[](size_t i) {
    return (i == 0) ? w : (i == 1) ? x : (i == 2) ? y : z;
  }
};

///////////////////////////////////////////////////////////
// IMU (Accels, Gyros)
///////////////////////////////////////////////////////////

struct imu_t {
  vec_t a,g;
  float temperature;
  bool ok;
  uint32_t ts;
};

struct imu_raw_t {
  vec_raw_t a,g;
  int16_t temperature; // lsm6dsox is only int16_t
  bool ok;
  uint32_t ts;
};


///////////////////////////////////////////////////////////
// Vector Message
///////////////////////////////////////////////////////////

struct vec_msg_t {
  float x, y, z;
  bool ok;  // error?

  inline const float magnitude() const { return sqrtf(x * x + y * y + z * z); }

  bool normalize() {
    float n = 1.0f / magnitude();
    if (std::isinf(n)) return false;

    x *= n;
    y *= n;
    z *= n;

    return true;
  }

  float operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};

struct vec_msg_raw_t {
  int16_t x, y, z;
  bool ok;  // error?

  int16_t operator[](size_t i) {
    return (i == 0) ? x : (i == 1) ? y : z;
  }
};


///////////////////////////////////////////////////////////
// Pressure / Temperature
///////////////////////////////////////////////////////////

struct pt_raw_t {
  int32_t pressure, temperature;
  bool ok;

  int32_t operator[](size_t i) {
    return (i == 0) ? pressure : temperature;
  }
};

struct pt_t {
  float pressure, temperature;
  bool ok;

  float operator[](size_t i) {
    return (i == 0) ? pressure : temperature;
  }
};


} // namespace sensors
} // namespace gci