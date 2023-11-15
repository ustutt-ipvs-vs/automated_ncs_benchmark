#include <fstream>
#include "ReceiverConfig.h"
#include "../nlohmann/json.hpp"

const std::string &ReceiverConfig::getReceiverAddress() const {
    return receiverAddress;
}

const std::string &ReceiverConfig::getPendulumType() const {
    return pendulumType;
}

const std::string &ReceiverConfig::getSerialDeviceName() const {
    return serialDeviceName;
}

bool ReceiverConfig::isAutomaticallyFindSerialDevice() const {
    return automaticallyFindSerialDevice;
}

int ReceiverConfig::getTimeBetweenPausesMillis() const {
    return timeBetweenPausesMillis;
}

int ReceiverConfig::getPauseDurationMillis() const {
    return pauseDurationMillis;
}

bool ReceiverConfig::isDoPauses() const {
    return doPauses;
}

std::string ReceiverConfig::toString() const {
    std::string result = "ReceiverConfig:\n";
    result += "receiverAddress: " + receiverAddress + "\n";
    result += "pendulumType: " + pendulumType + "\n";
    result += "serialDeviceName: " + serialDeviceName + "\n";
    result += "automaticallyFindSerialDevice: " + std::to_string(automaticallyFindSerialDevice) + "\n";
    result += "doPauses: " + std::to_string(doPauses) + "\n";
    result += "timeBetweenPausesMillis: " + std::to_string(timeBetweenPausesMillis) + "\n";
    result += "pauseDurationMillis: " + std::to_string(pauseDurationMillis) + "\n";
    result += "swingUpBehavior: " + getSwingUpBehaviorString() + "\n";
    return result;
}

ReceiverConfig::ReceiverConfig(std::string filename) {
    std::ifstream configFile(filename);
    if (!configFile.good()) {
        throw std::runtime_error("Config file does not exist");
    }

    nlohmann::json configJson = nlohmann::json::parse(configFile);

    receiverAddress = configJson["receiverAddress"];
    pendulumType = configJson["pendulumType"];
    if(configJson.contains("serialDeviceName")) {
        serialDeviceName = configJson["serialDeviceName"];
    } else {
        serialDeviceName = "auto";
    }
    automaticallyFindSerialDevice = (serialDeviceName == "auto");

    if(configJson.contains("doPauses")) {
        doPauses = configJson["doPauses"];
    } else {
        doPauses = false;
    }

    if(configJson.contains("timeBetweenPausesMillis")) {
        timeBetweenPausesMillis = configJson["timeBetweenPausesMillis"];
    } else {
        timeBetweenPausesMillis = 20'000;
    }

    if(configJson.contains("pauseDurationMillis")) {
        pauseDurationMillis = configJson["pauseDurationMillis"];
    } else {
        pauseDurationMillis = 800;
    }

    if(configJson.contains("swingUpBehavior")) {
        std::string swingUpBehaviorString = configJson["swingUpBehavior"];
        if(swingUpBehaviorString == "swingUpAtStart"){
            swingUpBehavior = SWING_UP_AT_START;
        } else if(swingUpBehaviorString == "swingUpAtNewConfigIfCrashed"){
            swingUpBehavior = SWING_UP_AT_NEW_CONFIG_IF_CRASHED;
        } else if(swingUpBehaviorString == "crashAndSwingUpAtNewConfig"){
            swingUpBehavior = CRASH_AND_SWING_UP_AT_NEW_CONFIG;
        } else if(swingUpBehaviorString == "noSwingUp"){
            swingUpBehavior = NO_SWING_UP;
        } else{
            throw std::runtime_error("Unknown swingUpBehavior: " + swingUpBehaviorString);
        }
    } else {
        swingUpBehavior = NO_SWING_UP;
    }
}

int ReceiverConfig::getMotorMaxRPM() const {
    if(pendulumType == "oldPendulum"){
        return 20 * 60;
    } else if(pendulumType == "newPendulum"){
        return 15 * 60;
    } else {
        throw std::runtime_error("Unknown pendulum type: " + pendulumType);
    }
}

double ReceiverConfig::getRevolutionsPerTrack() const {
    if(pendulumType == "oldPendulum"){
        return 20.06;
    } else if(pendulumType == "newPendulum"){
        return 15.00;
    } else{
        throw std::runtime_error("Unknown pendulum type: " + pendulumType);
    }
}

ReceiverConfig::SwingUpBehavior ReceiverConfig::getSwingUpBehavior() const {
    return swingUpBehavior;
}

std::string ReceiverConfig::getSwingUpBehaviorString() const {
    return swingUpBehaviorStrings[swingUpBehavior];
}



