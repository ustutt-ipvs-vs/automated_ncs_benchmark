/**
 * This is a class representing a JSON multi config file for the sender.
 * The configuration options in this file are targeted at running multiple
 * MPTB instances sequentially.
 *
 * For JSON parsing, the nlohmann/json library is used.
 *
 * An example config file:
 *  {
 *   "mptbSequence": [
 *      {
 *        "durationMinutes": 5,
 *        "b": 200.0,
 *        "r": 35.0,
 *        "numThresholds": 3,
 *        "thresholds": [-20000000, -200, -400],
 *        "prioMapping": [0, 1, 2, 7],
 *        "costs": [1, 1, 1, 0]
 *      },
 *      {
 *        "durationMinutes": 5,
 *        "b": 5000.0,
 *        "r": 350.0,
 *        "numThresholds": 2,
 *        "thresholds": [-20000, -40000],
 *        "prioMapping": [0, 1, 2],
 *        "costs": [2, 1, 0]
 *      }
 *   ],
 *   "historySize": 100,
 *   "bias": 0,
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
 * */

#ifndef SENDER_RECEIVER_LINUX_SENDERMULTICONFIG_H
#define SENDER_RECEIVER_LINUX_SENDERMULTICONFIG_H


#include <vector>
#include <string>
#include "MPTBSubConfig.h"
#include "../Parameters/Parameters.h"

class SenderMultiConfig {
private:
    std::vector<MPTBSubConfig> mptbSubConfigs;

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
    std::vector<int> networkDelaysPerPrio;

public:
    const std::vector<int> &getSamplingPeriods() const;

    float getSamplingPeriodSensitivityFactor() const;

    float getSamplingPeriodSensitivityOffset() const;

    int getHistorySize() const;

    int getBias() const;

    const std::string &getSerialDeviceName() const;

    bool isAutomaticallyFindSerialDevice() const;

    const std::string &getReceiverAddress() const;

    SenderMultiConfig(std::string filename);

    std::string toString() const;

    const std::vector<MPTBSubConfig> &getMptbSubConfigs() const;

    const std::vector<int> &getNetworkDelaysPerPrio() const;

    SendTriggeringApproach getSendTriggeringApproach() const;

    float getAngleTransmissionThreshold() const;

    std::string getTeensyInitializationString();



};

#endif //SENDER_RECEIVER_LINUX_SENDERMULTICONFIG_H
