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
 *   "pauseDurationMillis": 800,
 *   "swingUpBehavior": "swingUpAtStart"
 * }
 *
 * The pendulum types available are:
 * - oldPendulum: max RPM = 20 * 60, revolutionsPerTrack = 20.06
 * - newPendulum: max RPM = 15 * 60, revolutionsPerTrack = 15.00
 *
 * The swingUpBehavior options are:
 * - swingUpAtStart: swing up the pendulum at the start of the program
 * - swingUpAtNewConfigIfCrashed: swing up the pendulum at every new config if the pendulum crashed
 * - crashAndSwingUpAtNewConfig: intentionally crash the pendulum at every new config and swing it up again
 * - noSwingUp: don't swing up the pendulum
 *
 * The serialDeviceName can be set to "auto" to automatically find the a Teensy device.
 * It is also possible to specify the device name manually, e.g. "/dev/ttyACM0".
 */
class ReceiverConfig {
public:
    enum SwingUpBehavior {
        SWING_UP_AT_START,
        SWING_UP_AT_NEW_CONFIG_IF_CRASHED,
        CRASH_AND_SWING_UP_AT_NEW_CONFIG,
        NO_SWING_UP
    };

private:
    std::string receiverAddress;
    std::string pendulumType;
    std::string serialDeviceName;
    bool automaticallyFindSerialDevice;
    bool doPauses;
    int timeBetweenPausesMillis;
    int pauseDurationMillis;
    enum SwingUpBehavior swingUpBehavior;

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

    SwingUpBehavior getSwingUpBehavior() const;
};


#endif //SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H
