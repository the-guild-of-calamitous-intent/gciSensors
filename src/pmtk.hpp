/**************************************\
 * The MIT License (MIT)
 * Copyright (c) 2022 Kevin Walchko
 * see LICENSE for full details
\**************************************/
#pragma once

#include <stdint.h>

// CR \r
// LF \n
// <CR><LF> \r\n
// $[....]<CR><LF>

/////////////////////////////////////////////////////////////////////////////
// PMTK001 ACK
// PMTK01,CMD,FLAG*CHECKSUM
// FLAG: 0-invalid, 1-unsupported, 2-valid,failed, 3-valid,success
// $PMTK001,604,3*32<CR><LF>
//
// $PMTK001,314,3*36 -> (314) set nema output, PMTK::RMCGGA
// $PMTK001,220,3*30 -> (220) set pos fix, PMTK::UPDATE_1HZ
/////////////////////////////////////////////////////////////////////////////

#if defined(__USE_PMTK__)

namespace PMTK {
// PMTK commands
// https://www.sparkfun.com/datasheets/GPS/Modules/PMTK_Protocol.pdf
constexpr uint8_t UPDATE_1HZ[] = "$PMTK220,1000*1F\r\n"; //  1 Hz
constexpr uint8_t UPDATE_2HZ[] = "$PMTK220,500*2B\r\n";  //  2 Hz
constexpr uint8_t UPDATE_5HZ[] = "$PMTK220,200*2C\r\n";  //  5 Hz ... invalid?
// constexpr uint8_t UPDATE_10HZ[] = "$PMTK220,100*2F\r\n"; // 10 Hz ... invalid? must > 200, only 100

// power modes
constexpr uint8_t FULL_POWER[] = "$PMTK225,0*2B\r\n"; // full pwr/continuous

// Position fix update rates
constexpr uint8_t FIX_CTL_1HZ[] = "$PMTK300,1000,0,0,0,0*1C\r\n"; // 1 Hz output
constexpr uint8_t FIX_CTL_5HZ[] = "$PMTK300,200,0,0,0,0*2F\r\n";  // 5 Hz output

constexpr uint8_t BAUD_115200[] = "$PMTK251,115200*1F\r\n"; // 115200 bps
constexpr uint8_t BAUD_57600[] = "$PMTK251,57600*2C\r\n";   //  57600 bps
constexpr uint8_t BAUD_9600[] = "$PMTK251,9600*17\r\n";     //   9600 bps

constexpr uint8_t ANTENNA[] = "$PGCMD,33,1*6C\r\n"; // request for updates on antenna status
constexpr uint8_t NOANTENNA[] = "$PGCMD,33,0*6D\r\n"; // don't show antenna status messages

constexpr uint8_t ENABLE_SBAS[] = "$PMTK313,1*2E\r\n"; // Enable search for SBAS satellite (only works with 1Hz < output rate)
constexpr uint8_t ENABLE_WAAS[] = "$PMTK301,2*2E\r\n"; // WAAS for DGPS correction data

constexpr uint8_t GLL[] = "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // only the GLL sentence
constexpr uint8_t RMC[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // only the RMC sentence
constexpr uint8_t VTG[] = "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // only the VTG
constexpr uint8_t GGA[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // just the GGA
constexpr uint8_t GSA[] = "$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // just the GSA
constexpr uint8_t GSV[] = "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // just the GSV
constexpr uint8_t RMCGGA[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; // RMC and GGA
constexpr uint8_t RMCGGAGSA[] = "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; // RMC, GGA and GSA
} // end namespace

#endif // use_pmtk