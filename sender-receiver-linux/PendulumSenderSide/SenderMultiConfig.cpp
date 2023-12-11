#include <fstream>
#include "../nlohmann/json.hpp"
#include "MPTBSubConfig.h"
#include "SenderMultiConfig.h"


SenderMultiConfig::SenderMultiConfig(std::string filename) {
    std::ifstream configFile(filename);
    if (!configFile.good()) {
        throw std::runtime_error("Config file does not exist");
    }

    nlohmann::json configJson = nlohmann::json::parse(configFile);
    configFile.close();

    if (!configJson.contains("mptbSequence")) {
        throw std::runtime_error("Attribute mptbSequence array missing in config file.");
    }

    for (const nlohmann::json& subConfigJson : configJson["mptbSequence"]) {
        MPTBSubConfig subConfig(subConfigJson);
        mptbSubConfigs.emplace_back(subConfig);
    }

    if (configJson.contains("historySize")) {
        historySize = configJson["historySize"];
    } else {
        historySize = 100; // default value
    }
    if (configJson.contains("bias")) {
        bias = configJson["bias"];
    } else {
        bias = 0; // default value
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

    if(historySize < 0){
        throw std::runtime_error("historySize must be at least 0");
    }

    if(configJson.contains("serialDeviceName")) {
        serialDeviceName = configJson["serialDeviceName"];
    } else {
        serialDeviceName = "auto";
    }
    automaticallyFindSerialDevice = (serialDeviceName == "auto");
    receiverAddress = configJson["receiverAddress"];
}

int SenderMultiConfig::getHistorySize() const {
    return historySize;
}

const std::vector<int> &SenderMultiConfig::getSamplingPeriods() const {
    return samplingPeriods;
}

const std::string &SenderMultiConfig::getSerialDeviceName() const {
    return serialDeviceName;
}

bool SenderMultiConfig::isAutomaticallyFindSerialDevice() const {
    return automaticallyFindSerialDevice;
}

const std::string &SenderMultiConfig::getReceiverAddress() const {
    return receiverAddress;
}

const std::vector<MPTBSubConfig> &SenderMultiConfig::getMptbSubConfigs() const {
    return mptbSubConfigs;
}

std::string SenderMultiConfig::toString() const {
    std::string result = "SenderMultiConfig:\n";
    result += "historySize: " + std::to_string(historySize) + "\n";
    result += "bias: " + std::to_string(bias) + "\n";
    result += "samplingPeriods: ";
    for(int samplingPeriod : samplingPeriods){
        result += std::to_string(samplingPeriod) + " ";
    }
    result += "\n";
    result += "samplingPeriodSensitivityFactor: " + std::to_string(samplingPeriodSensitivityFactor) + "\n";
    result += "samplingPeriodSensitivityOffset: " + std::to_string(samplingPeriodSensitivityOffset) + "\n";
    result += "serialDeviceName: " + serialDeviceName + "\n";
    result += "receiverAddress: " + receiverAddress + "\n";
    result += "mptbSubConfigs:\n";
    for(const MPTBSubConfig& subConfig : mptbSubConfigs){
        result += subConfig.toString() + "\n";
    }
    return result;
}

int SenderMultiConfig::getBias() const {
    return bias;
}

float SenderMultiConfig::getSamplingPeriodSensitivityFactor() const {
    return samplingPeriodSensitivityFactor;
}

float SenderMultiConfig::getSamplingPeriodSensitivityOffset() const {
    return samplingPeriodSensitivityOffset;
}


