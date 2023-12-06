#ifndef SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H
#define SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H

#include <string>
#include <vector>

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
 *   "swingUpBehavior": "swingUpAtStart",
 *   "sailType": "sail14",
 *   "controllerKVector": [1.0, 2.0, 3.0, 4.0],
 *   "controllerIntegratorParam": 1.0,
 *   "RMatrixDiagonalValue": 1.0,
 *   "Q0MatrixDiagonalValue": 1.0,
 *   "sigmaSquare": 1.0
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
 * The sailType options are:
 * - noSail: no sail
 * - sail10: 10cm long sail
 * - sail14: 14cm long sail
 * - sail17: 17cm long sail
 * - sail20: 20cm long sail
 * Default is noSail.
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
    const std::vector<std::string> swingUpBehaviorStrings = {
            "swingUpAtStart",
            "swingUpAtNewConfigIfCrashed",
            "crashAndSwingUpAtNewConfig",
            "noSwingUp"
    };

    std::string receiverAddress;
    std::string pendulumType;
    std::string serialDeviceName;
    bool automaticallyFindSerialDevice;
    bool doPauses;
    int timeBetweenPausesMillis;
    int pauseDurationMillis;
    enum SwingUpBehavior swingUpBehavior;
    std::string pendulumSailType;

    std::vector<float> controllerKVector;
    float controllerIntegratorParam;
    float RMatrixDiagonalValue;
    float Q0MatrixDiagonalValue;
    float sigmaSquare;

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

    std::string getSwingUpBehaviorString() const;

    const std::string &getPendulumSailType() const;

    float getSwingUpDistanceFactor() const;

    float getSwingUpSpeedFactor() const;

    float getSwingUpAccelerationFactor() const;


    const std::vector<float> &getControllerKVector() const;

    float getControllerIntegratorParam() const;

    float getRMatrixDiagonalValue() const;

    float getQ0MatrixDiagonalValue() const;

    float getSigmaSquare() const;

    /**
     * Returns a string containing all the parameters that are used by the Kalman filter and the controller.
     * The string is formatted as follows:
     * controllerKVector[0];controllerKVector[1];controllerKVector[2];controllerKVector[3];controllerIntegratorParam;RMatrixDiagonalValue;Q0MatrixDiagonalValue;sigmaSquare;
     *
     * This string is used to transfer to initial config to the Teensy microcontroller on initialization.
     */
    std::string getKalmanAndControllerParameterString();
};

#endif //SENDER_RECEIVER_LINUX_RECEIVERCONFIG_H
