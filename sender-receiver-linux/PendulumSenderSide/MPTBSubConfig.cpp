//
// Created by ipvsuser on 10/18/23.
//

#include "MPTBSubConfig.h"

double MPTBSubConfig::getDurationMinutes() const {
    return durationMinutes;
}

double MPTBSubConfig::getB() const {
    return b;
}

double MPTBSubConfig::getR() const {
    return r;
}

int MPTBSubConfig::getNumThresholds() const {
    return numThresholds;
}

const std::vector<double> &MPTBSubConfig::getThresholds() const {
    return thresholds;
}

const std::vector<int> &MPTBSubConfig::getPrioMapping() const {
    return prioMapping;
}

const std::vector<double> &MPTBSubConfig::getCosts() const {
    return costs;
}

MPTBSubConfig::MPTBSubConfig(nlohmann::json configJson) {
    durationMinutes = configJson["durationMinutes"];
    b = configJson["b"];
    r = configJson["r"];
    numThresholds = configJson["numThresholds"];

    thresholds = configJson["thresholds"].get<std::vector<double>>();
    prioMapping = configJson["prioMapping"].get<std::vector<int>>();
    costs = configJson["costs"].get<std::vector<double>>();

    // Integrity checks:
    if (numThresholds < 1) {
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

std::string MPTBSubConfig::toString() const {
    std::string result = "MPTBSubConfig:\n";
    result += "durationMinutes: " + std::to_string(durationMinutes) + "\n";
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
    return result;
}

