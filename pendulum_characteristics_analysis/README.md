# How To

1. Upload the `teensy_encoder_serial_readout.ino` sketch onto a Teensy microcontroller that is connected to a rotary encoder on pin 2 an 3.
2. Plug in the Teensy in the computer. The teensy immediately starts writing the raw encoder values to the Serial interface every millisecond.
3. Run the `teensy_encoder_log.sh` script to receive the values from the serial interface and write them to a log file.
4. Do your measurements.
5. Stop the script with `Ctrl+C` when you are done measuring.
6. Use the `analyze_trace.py` script to analyze the created log file.
