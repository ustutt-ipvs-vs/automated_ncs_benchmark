/**
 * This is a class representing a JSON config file for the sender.
 * The configuration options in this file are targeted at MPTB scheduling.
 *
 * For JSON parsing, the nlohmann/json library is used.
 *
 * An example config file:
 *  {
 *   "b": 200.0,
 *   "r": 30.0,
 *   "numThresholds": 4,
 *   "thresholds": [0, -200, -400],
 *   "prioMapping": [0, 1, 2, 7],
 *   "costs": [1, 1, 1, 0],
 *   "bias": 0,
 *   "sendTriggeringApproach": "slidingWindow",
 *   "historySize": 100,
 *   "samplingPeriods": [100, 90, 80, 70, 60, 50, 40, 30, 20, 10],
 *   "samplingPeriodSensitivityFactor": 0.5,
 *   "samplingPeriodSensitivityOffset": 2.0,
 *   "initialPriorityClass": 0,
 *   "serialDeviceName": "/dev/ttyACM0",
 *   "receiverAddress": "10.0.1.3",
 *   "networkDelaysPerPrio": [2, 4, 6, 8, 10, 12, 14, 16]
 * }
 *
 *  The default values are, if not specified otherwise:
 *  sendTriggeringApproach = "slidingWindow"
 *  historySize = 100
 *  bias = 0
 *  samplingPeriods = [100, 90, 80, 70, 60, 50, 40, 30, 20, 10]
 *  samplingPeriodSensitivityFactor: 1.0
 *  samplingPeriodSensitivityOffset: 0.0
 *  initialPriorityClass = 0
 *  serialDeviceName = "auto"   // automatically find the device by scanning /dev/ttyACM*
 *  "networkDelaysPerPrio": [0, 0, 0, 0, 0, 0, 0, 0]
 *
 * The serialDeviceName can be set to "auto" to automatically find the a Teensy device.
 * It is also possible to specify the device name manually, e.g. "/dev/ttyACM0".
 *
 * If sendTriggeringApproach is set to "simpleThreshold", the following parameters are required:
 * "angleTransmissionThreshold": 1.23
 * where the number is the angle in degrees. The parameters "historySize", "samplingPeriods",
 * "samplingPeriodSensitivityFactor", and "samplingPeriodSensitivityOffset" are not required in this case.
 * */

#ifndef SENDER_RECEIVER_LINUX_SENDERCONFIG_H
#define SENDER_RECEIVER_LINUX_SENDERCONFIG_H


#include <vector>
#include <string>
#include "../Parameters/Parameters.h"

class SenderConfig {
private:

    // MPTB parameters:
    double b;
    double r;
    int numThresholds;
    int initialPriorityClass;
    std::vector<double> thresholds;
    std::vector<int> prioMapping;
    std::vector<double> costs;
    std::vector<int> networkDelaysPerPrio;

    // Teensy parameters:
    int bias;
    std::string serialDeviceName;
    bool automaticallyFindSerialDevice;
    SendTriggeringApproach sendTriggeringApproach;
    // SLIDING_WINDOW parameters:
    int historySize;
    std::vector<int> samplingPeriods;
    float samplingPeriodSensitivityFactor;
    float samplingPeriodSensitivityOffset;
    // SIMPLE_THRESHOLD parameters:
    float angleTransmissionThreshold;

    // Network parameters:
    std::string receiverAddress;

public:
    const std::vector<int> &getSamplingPeriods() const;

    int getHistorySize() const;

    float getSamplingPeriodSensitivityFactor() const;

    float getSamplingPeriodSensitivityOffset() const;

    int getBias() const;

    const std::string &getSerialDeviceName() const;

    bool isAutomaticallyFindSerialDevice() const;

    const std::string &getReceiverAddress() const;

    SenderConfig(std::string filename);

    double getB() const;

    double getR() const;

    int getNumThresholds() const;

    const std::vector<double> &getThresholds() const;

    const std::vector<int> &getPrioMapping() const;

    const std::vector<double> &getCosts() const;

    const std::vector<int> &getNetworkDelaysPerPrio() const;

    int getInitialPriorityClass() const;

    std::string toString() const;

    SendTriggeringApproach getSendTriggeringApproach() const;

    float getAngleTransmissionThreshold() const;

    std::string getTeensyInitializationString();

};

#endif //SENDER_RECEIVER_LINUX_SENDERCONFIG_H
