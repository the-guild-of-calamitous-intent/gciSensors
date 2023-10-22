#!/bin/bash

# grab 1st arg as the UF2 file to load
UF2=$1

echo "Loading ${UF2} to pi pico\n"

# -v: verify load
# -f: force device into BOOTSEL mode, reboot when done
picotool load "${UF2}" -v -f