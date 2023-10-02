/**
 * This class offers a method which automatically finds the serial device of a Teensy board.
 *
 * For this purpose, it scans the /dev/ttyACM* devices and looks at their properties. The first device with
 * the property "ID_VENDOR_ID=Teensyduino" is chosen.
 */

#ifndef SENDER_RECEIVER_LINUX_TEENSYPORTDETECTOR_H
#define SENDER_RECEIVER_LINUX_TEENSYPORTDETECTOR_H


#include <string>

class TeensyPortDetector {
public:
    static std::string findTeensySerialDevice();

private:
    static std::vector<std::string> getAllSerialDeviceNames();

    static std::string runShellCommand(const std::string &command);
};


#endif //SENDER_RECEIVER_LINUX_TEENSYPORTDETECTOR_H
