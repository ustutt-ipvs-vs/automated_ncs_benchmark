#!/bin/bash
# Script to log encoder values from Teensy serial to file.

# Check if filename is given
if [ $# -eq 0 ]
  then
    echo "No filename given. Usage: bash teensy_encoder_log.sh <filename> [<serialDevice>]"
    exit 1
fi

filename=$1
serialDevice=${2:-/dev/ttyACM0}

# Check if serial device is connected (file serialDevice exists)
if [ ! -e "$serialDevice" ]
  then
    echo "Serial device $serialDevice not found. Check if device is connected."
    echo "Usage: bash teensy_encoder_log.sh <filename> [<serialDevice>]"
    echo "If serialDevice is not given, /dev/ttyACM0 is used."
    exit 1
fi

# Write first line to file
echo "timeMicros;encoderValue" > $filename

echo "Logging to $filename. Press Ctrl+C to stop."

# Read from serial device and append to file
cat /dev/ttyACM0 >> $filename
