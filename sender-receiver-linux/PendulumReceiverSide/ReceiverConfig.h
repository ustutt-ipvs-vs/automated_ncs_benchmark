#ifndef SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H
#define SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H

#include <string>

/**
 * This class is used to read the configuration JSON file for the receiver.
 * Example config file:
 * {
 *   "receiverAddress": "10.0.0.3",
 *   "pendulumType": "newPendulum",
 *   "serialDeviceName": "auto",
 *   "doPauses": true,
 *   "timeBetweenPausesMillis": 20000,
 *   "pauseDurationMillis": 800
 * }
 *
 * The pendulum types available are:
 * - oldPendulum: max RPM = 20 * 60, revolutionsPerTrack = 20.06
 * - newPendulum: max RPM = 15 * 60, revolutionsPerTrack = 15.00
 *
 * The serialDeviceName can be set to "auto" to automatically find the a Teensy device.
 * It is also possible to specify the device name manually, e.g. "/dev/ttyACM0".
 */
class ReceiverConfig {
private:
    std::string receiverAddress;
    std::string pendulumType;
    std::string serialDeviceName;
    bool automaticallyFindSerialDevice;
    bool doPauses;
    int timeBetweenPausesMillis;
    int pauseDurationMillis;

public:
    const std::string &getReceiverAddress() const;

    const std::string &getPendulumType() const;

    const std::string &getSerialDeviceName() const;

    std::string toString() const;

    ReceiverConfig(std::string filename);

    bool isAutomaticallyFindSerialDevice() const;

    bool isDoPauses() const;

    int getTimeBetweenPausesMillis() const;

    int getPauseDurationMillis() const;

    int getMotorMaxRPM() const;

    double getRevolutionsPerTrack() const;


};


#endif //SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H
