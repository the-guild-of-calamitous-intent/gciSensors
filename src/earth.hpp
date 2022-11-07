/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2016 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include "units.hpp"
#include <algorithm> // std::min
#include <math.h>

namespace Earth {

constexpr float INV_FLATTENING = 298.257223563f;
constexpr float FLATTENING = 1.0f / INV_FLATTENING;
constexpr float SEMI_MAJOR_AXIS_M = 6378137.0f;                   // m
constexpr float SEMI_MAJOR_AXIS_KM = SEMI_MAJOR_AXIS_M / 1000.0f; // Km
constexpr float STD_PRESSURE_PA = 101325.0f;                      // Pa
constexpr float SPIN_RATE_RPS = 7.2921150e-5f;                    // rad / sec
constexpr float G0 = 9.7803253359; // Gravity [m/sec^2]

using std::min;

// change to lla_t??
struct gps_t {
  float lat, lon, alt;
};

/*
WGS84 is used in GPS which are geodetic coordinates.

Geodetic coordinates are a type of curvilinear orthogonal coordinate
system used in geodesy based on a reference ellipsoid. They include
geodetic latitude (north/south) phi, longitude (east/west) lambda, and
ellipsoidal height h (also known as geodetic height). The triad
is also known as Earth ellipsoidal coordinates
https://en.wikipedia.org/wiki/Geodetic_coordinates

https://en.wikipedia.org/wiki/Earth_radius
https://en.wikipedia.org/wiki/Earth_ellipsoid
https://en.wikipedia.org/wiki/World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84

Geodetic coordinates: latitude, longitude which are reported by GPS

ECEF: a geocentric coordinate system, which is just a 3D cartesian
      spatial reference system with the origin at the Earth's
      center of mass.
ref: https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system

rf: inverse flattening
f: flattening
a: equitorial radius or semi-major axis [m]
b: polar radius or semi-minor axis [m]
e: eccentricity of Earth ellipsoid
r: spherical radius [m]
rotation_period: Earth's rotation rate [seconds]
rate: Rotation rate of Earth [rad/s]
sf: Schuller frequency
*/

struct WGS84_t {
  /*
  Returns the haversine (or great circle) distance between
  2 sets of GPS coordinates. This appears to work really well.

  a: (lat, lon) in deg
  b: (lat, lon) in deg
  */
  float haversine(const gps_t &a, const gps_t &b) {
    const float alat = a.lat * Units::deg2rad;
    const float blat = b.lat * Units::deg2rad;
    const float R = SEMI_MAJOR_AXIS_M;
    const float dlat = (b.lat - a.lat) * Units::deg2rad;
    const float dlon = (b.lon - a.lon) * Units::deg2rad;
    const float m = pow(sin(dlat * 0.5), 2) +
                    cos(alat) * cos(blat) * pow(sin(dlon * 0.5), 2);
    return R * 2.0f * asin(min(1.0f, sqrt(m)));
  }

  /*
  Returns the geocentric radius based on WGS84
  lat: latitude in deg
  */
  float radius(float lat) {
    const float a = SEMI_MAJOR_AXIS_M;
    const float b = a - a * FLATTENING;
    lat *= Units::deg2rad;
    const float num = pow(a * a * cos(lat), 2) + pow(b * b * sin(lat), 2);
    const float den = pow(a * cos(lat), 2) + pow(b * sin(lat), 2);
    return sqrtf(num / den);
  }

  /*
  Based off the Oxford reference for the gravity formula at sealevel.
  https://www.oxfordreference.com/view/10.1093/oi/authority.20110803100007626

  Also the WGS84 has a newer model, but it is more computationally
  intensive and only differs from this one by 0.68 um/s^2
  https://en.wikipedia.org/wiki/Gravity_of_Earth

  lat: latitude [decimal deg], North is positive and South is negative
  */
  float gravity(float lat) {
    lat *= Units::deg2rad;
    return G0 * (1.0f + 0.0053024f * pow(sin(lat), 2) -
                 0.0000058 * pow(sin(2 * lat), 2));
  }
};

/*
Not sure this is really useful.
*/
struct WMM_t {
  float dummy;
};

} // namespace Earth