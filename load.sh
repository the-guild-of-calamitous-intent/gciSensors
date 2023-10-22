#!/bin/bash

# UF2="$(ls *.uf2)"
UF2=$1

echo "Loading ${UF2} to pi pico\n"
picotool load "${UF2}" -v -f