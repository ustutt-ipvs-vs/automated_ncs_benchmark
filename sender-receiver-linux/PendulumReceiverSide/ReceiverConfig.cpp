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
    result += "sailType: " + pendulumSailType + "\n";

    result += "controllerKVector: [";
    for(float i : controllerKVector){
        result +=  std::to_string(i) + ", ";
    }
    result += "]\n";
    result += "controllerIntegratorParam: " + std::to_string(controllerIntegratorParam) + "\n";
    result += "RMatrixDiagonalValue: " + std::to_string(RMatrixDiagonalValue) + "\n";
    result += "Q0MatrixDiagonalValue: " + std::to_string(Q0MatrixDiagonalValue) + "\n";
    result += "sigmaSquare: " + std::to_string(sigmaSquare) + "\n";

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
            std::string optionsString;
            for(const std::string& option : swingUpBehaviorStrings){
                optionsString += option;
                if (option != swingUpBehaviorStrings.back()){
                    optionsString += ", ";
                }
            }
            throw std::runtime_error("Unknown swingUpBehavior: " + swingUpBehaviorString
             + " The following options are available: " + optionsString);
        }
    } else {
        swingUpBehavior = NO_SWING_UP;
    }

    if(configJson.contains("sailType")) {
        pendulumSailType = configJson["sailType"];
    } else {
        pendulumSailType = "noSail";
    }

    if(configJson.contains("controllerKVector")) {
        controllerKVector = configJson["controllerKVector"].get<std::vector<float>>();;
    } else {
        controllerKVector = {3.6723, 13.5022, -74.6153, -19.8637};
    }

    if(configJson.contains("controllerIntegratorParam")) {
        controllerIntegratorParam = configJson["controllerIntegratorParam"];
    } else {
        controllerIntegratorParam = 5.0322;
    }

    if(configJson.contains("RMatrixDiagonalValue")) {
        RMatrixDiagonalValue = configJson["RMatrixDiagonalValue"];
    } else {
        RMatrixDiagonalValue = 1.0;
    }

    if(configJson.contains("Q0MatrixDiagonalValue")) {
        Q0MatrixDiagonalValue = configJson["Q0MatrixDiagonalValue"];
    } else {
        Q0MatrixDiagonalValue = 1.0;
    }

    if(configJson.contains("sigmaSquare")) {
        sigmaSquare = configJson["sigmaSquare"];
    } else {
        sigmaSquare = 1.0;
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
    return swingUpBehaviorStrings.at(swingUpBehavior);
}

const std::string &ReceiverConfig::getPendulumSailType() const {
    return pendulumSailType;
}

float ReceiverConfig::getSwingUpDistanceFactor() const {
    if(pendulumSailType == "noSail"){
        return 0.09;
    } if(pendulumSailType == "sail10"){
        return 0.105; // TODO: measure (dummy value)
    } if(pendulumSailType == "sail14"){
        return 0.14;
    } if(pendulumSailType == "sail17"){
        return 0.10; // // TODO: measure (dummy value)
    } if(pendulumSailType == "sail20"){
        return 0.10; // TODO: measure (dummy value)
    } else {
        throw std::runtime_error("Unknown pendulum sail type: " + pendulumSailType);
    }
}

float ReceiverConfig::getSwingUpSpeedFactor() const {
    if(pendulumSailType == "noSail"){
        return 1.0;
    } if(pendulumSailType == "sail10"){
        return 1.0; // TODO: measure (dummy value)
    } if(pendulumSailType == "sail14"){
        return 1.2;
    } if(pendulumSailType == "sail17"){
        return 1.0; // // TODO: measure (dummy value)
    } if(pendulumSailType == "sail20"){
        return 1.0; // TODO: measure (dummy value)
    } else {
        throw std::runtime_error("Unknown pendulum sail type: " + pendulumSailType);
    }
}

float ReceiverConfig::getSwingUpAccelerationFactor() const {
    if(pendulumSailType == "noSail"){
        return 1.0;
    } if(pendulumSailType == "sail10"){
        return 1.0; // TODO: measure (dummy value)
    } if(pendulumSailType == "sail14"){
        return 1.1;
    } if(pendulumSailType == "sail17"){
        return 1.0; // // TODO: measure (dummy value)
    } if(pendulumSailType == "sail20"){
        return 1.0; // TODO: measure (dummy value)
    } else {
        throw std::runtime_error("Unknown pendulum sail type: " + pendulumSailType);
    }
}

const std::vector<float> &ReceiverConfig::getControllerKVector() const {
    return controllerKVector;
}

float ReceiverConfig::getControllerIntegratorParam() const {
    return controllerIntegratorParam;
}

float ReceiverConfig::getRMatrixDiagonalValue() const {
    return RMatrixDiagonalValue;
}

float ReceiverConfig::getQ0MatrixDiagonalValue() const {
    return Q0MatrixDiagonalValue;
}

float ReceiverConfig::getSigmaSquare() const {
    return sigmaSquare;
}

std::string ReceiverConfig::getKalmanAndControllerParameterString() {
    std::string result;
    for(float i : controllerKVector){
        result +=  std::to_string(i) + ";";
    }
    result += std::to_string(controllerIntegratorParam) + ";";
    result += std::to_string(RMatrixDiagonalValue) + ";";
    result += std::to_string(Q0MatrixDiagonalValue) + ";";
    result += std::to_string(sigmaSquare) + ";";
    return result;
}




