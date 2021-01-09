#!/bin/bash
pio remote run --upload-port /dev/ttyACM2 --target upload &
sleep 6 
pio remote run --upload-port /dev/ttyACM1 --target upload &


