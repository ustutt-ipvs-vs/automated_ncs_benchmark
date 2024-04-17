#include <fstream>
#include "SenderConfig.h"
#include "../nlohmann/json.hpp"

double SenderConfig::getB() const {
    return b;
}

double SenderConfig::getR() const {
    return r;
}

int SenderConfig::getNumThresholds() const {
    return numThresholds;
}

const std::vector<double> &SenderConfig::getThresholds() const {
    return thresholds;
}

const std::vector<int> &SenderConfig::getPrioMapping() const {
    return prioMapping;
}

const std::vector<double> &SenderConfig::getCosts() const {
    return costs;
}

SenderConfig::SenderConfig(std::string filename) {
    std::ifstream configFile(filename);
    if (!configFile.good()) {
        throw std::runtime_error("Config file does not exist");
    }

    nlohmann::json configJson = nlohmann::json::parse(configFile);

    b = configJson["b"];
    r = configJson["r"];
    numThresholds = configJson["numThresholds"];

    thresholds = configJson["thresholds"].get<std::vector<double>>();
    prioMapping = configJson["prioMapping"].get<std::vector<int>>();
    costs = configJson["costs"].get<std::vector<double>>();

    if (configJson.contains("networkDelaysPerPrio")) {
        if(configJson["networkDelaysPerPrio"].size() != 8) {
            throw std::runtime_error("networkDelaysPerPrio must have 8 elements");
        }
        networkDelaysPerPrio = configJson["networkDelaysPerPrio"].get<std::vector<int>>();
    } else {
        networkDelaysPerPrio = std::vector<int>(8, 0); // default value
    }

    if (configJson.contains("bias")) {
        bias = configJson["bias"];
    } else {
        bias = 0; // default value
    }

    if(configJson.contains("sendTriggeringApproach")) {
        if (configJson["sendTriggeringApproach"] == "slidingWindow") {
            sendTriggeringApproach = SLIDING_WINDOW;
        } else if (configJson["sendTriggeringApproach"] == "simpleThreshold") {
            sendTriggeringApproach = SIMPLE_THRESHOLD;
        } else {
            throw std::runtime_error(
                    "Missing or wrong config option: sendTriggeringApproach must be either slidingWindow or simpleThreshold");
        }
    } else {
        sendTriggeringApproach = SLIDING_WINDOW; // default value
    }

    if(sendTriggeringApproach == SLIDING_WINDOW){
        if (configJson.contains("historySize")) {
            historySize = configJson["historySize"];
        } else {
            historySize = 100; // default value
        }

        if(historySize < 0){
            throw std::runtime_error("historySize must be at least 0");
        }

        if (configJson.contains("samplingPeriods")) {
            samplingPeriods = configJson["samplingPeriods"].get<std::vector<int>>();
        } else {
            samplingPeriods = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10}; // default value
        }

        if (configJson.contains("samplingPeriodSensitivityFactor")) {
            samplingPeriodSensitivityFactor = configJson["samplingPeriodSensitivityFactor"];
        } else {
            samplingPeriodSensitivityFactor = 1.0; // default value
        }

        if (configJson.contains("samplingPeriodSensitivityOffset")) {
            samplingPeriodSensitivityOffset = configJson["samplingPeriodSensitivityOffset"];
        } else {
            samplingPeriodSensitivityOffset = 0.0; // default value
        }
    } else { // SIMPLE_THRESHOLD
        if (configJson.contains("angleTransmissionThreshold")) {
            angleTransmissionThreshold = configJson["angleTransmissionThreshold"];
        } else {
            throw std::runtime_error("angleTransmissionThreshold must be specified for SIMPLE_THRESHOLD approach");
        }
    }

    if (configJson.contains("initialPriorityClass")) {
        initialPriorityClass = configJson["initialPriorityClass"];
    } else {
        initialPriorityClass = 0; // default value
    }
    if(configJson.contains("serialDeviceName")) {
        serialDeviceName = configJson["serialDeviceName"];
    } else {
        serialDeviceName = "auto";
    }
    automaticallyFindSerialDevice = (serialDeviceName == "auto");
    receiverAddress = configJson["receiverAddress"];

    configFile.close();

    if(numThresholds < 1) {
        throw std::runtime_error("numThresholds must be at least 1");
    }

    if (thresholds.size() != numThresholds) {
        throw std::runtime_error("Number of thresholds is not numThresholds");
    }
    if (prioMapping.size() != numThresholds + 1) {
        throw std::runtime_error("Number of priorities does not match numThresholds + 1");
    }
    if (costs.size() != numThresholds + 1) {
        throw std::runtime_error("Number of costs does not match numThresholds + 1");
    }
}

int SenderConfig::getHistorySize() const {
    return historySize;
}

const std::vector<int> &SenderConfig::getSamplingPeriods() const {
    return samplingPeriods;
}

const std::string &SenderConfig::getSerialDeviceName() const {
    return serialDeviceName;
}

bool SenderConfig::isAutomaticallyFindSerialDevice() const {
    return automaticallyFindSerialDevice;
}

const std::string &SenderConfig::getReceiverAddress() const {
    return receiverAddress;
}

int SenderConfig::getInitialPriorityClass() const {
    return initialPriorityClass;
}

std::string SenderConfig::toString() const {
    std::string result = "SenderConfig:\n";
    result += "b: " + std::to_string(b) + "\n";
    result += "r: " + std::to_string(r) + "\n";
    result += "numThresholds: " + std::to_string(numThresholds) + "\n";
    result += "thresholds: ";
    for(double threshold : thresholds){
        result += std::to_string(threshold) + " ";
    }
    result += "\n";
    result += "prioMapping: ";
    for(int prio : prioMapping){
        result += std::to_string(prio) + " ";
    }
    result += "\n";
    result += "costs: ";
    for(double cost : costs){
        result += std::to_string(cost) + " ";
    }
    result += "\n";

    result += "networkDelaysPerPrio: ";
    for(int delay : networkDelaysPerPrio){
        result += std::to_string(delay) + " ";
    }
    result += "\n";

    result += "bias: " + std::to_string(bias) + "\n";

    if(sendTriggeringApproach == SLIDING_WINDOW) {
        result += "sendTriggeringApproach: SLIDING_WINDOW\n";
        result += "historySize: " + std::to_string(historySize) + "\n";
        result += "samplingPeriodSensitivityFactor: " + std::to_string(samplingPeriodSensitivityFactor) + "\n";
        result += "samplingPeriodSensitivityOffset: " + std::to_string(samplingPeriodSensitivityOffset) + "\n";
        result += "samplingPeriods: ";
        for(int period : samplingPeriods){
            result += std::to_string(period) + " ";
        }
        result += "\n";
    } else { // SIMPLE_THRESHOLD
        result += "sendTriggeringApproach: SIMPLE_THRESHOLD\n";
        result += "angleTransmissionThreshold: " + std::to_string(angleTransmissionThreshold) + "\n";
    }

    result += "serialDeviceName: " + serialDeviceName + "\n";
    result += "automaticallyFindSerialDevice: " + std::to_string(automaticallyFindSerialDevice) + "\n";
    result += "receiverAddress: " + receiverAddress + "\n";
    return result;
}

int SenderConfig::getBias() const {
    return bias;
}

float SenderConfig::getSamplingPeriodSensitivityFactor() const {
    return samplingPeriodSensitivityFactor;
}

float SenderConfig::getSamplingPeriodSensitivityOffset() const {
    return samplingPeriodSensitivityOffset;
}

const std::vector<int> &SenderConfig::getNetworkDelaysPerPrio() const {
    return networkDelaysPerPrio;
}

SendTriggeringApproach SenderConfig::getSendTriggeringApproach() const {
    return sendTriggeringApproach;
}

float SenderConfig::getAngleTransmissionThreshold() const {
    return angleTransmissionThreshold;
}

std::string SenderConfig::getTeensyInitializationString() {
    std::string teensyInitParams = "H:";
    if(sendTriggeringApproach == SLIDING_WINDOW) {
        // Create string of form "H:0;sensitivityFactor;sensitivityOffset;historySize;period1;period2;period3;...\n"
        teensyInitParams += "0;" // 0 encodes the SLIDING_WINDOW approach
                           + std::to_string(samplingPeriodSensitivityFactor) + ";"
                           + std::to_string(samplingPeriodSensitivityOffset) + ";"
                           + std::to_string(historySize) + ";";
        for (int period : samplingPeriods) {
            teensyInitParams += std::to_string(period) + ";";
        }
    } else {
        // Create string of form "H:1;angleTransmissionThreshold;\n"
        teensyInitParams += "1;" // 1 encodes the SIMPLE_THRESHOLD approach
                           + std::to_string(angleTransmissionThreshold) + ";";
    }
    teensyInitParams += "\n";
    return teensyInitParams;
}
