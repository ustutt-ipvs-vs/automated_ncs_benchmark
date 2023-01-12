//
// Created by david on 10.01.23.
//

#include "PendulumLogger.h"
#include <sstream>

PendulumLogger::PendulumLogger(std::string name) : Logger(name) {}

void PendulumLogger::log(unsigned long long packetCount, unsigned long long bytesSentTotal, std::string payload,
                         SchedulingInfoEntry *schedulingInfo) {
    time_point<system_clock> currentTime = system_clock::now();

    // payload must be of the form "1234;5678;\n"
    int pendulumSensorValue, samplingPeriodMillis;
    std::stringstream stringStream(payload);
    stringStream >> pendulumSensorValue;
    stringStream.ignore(); // skip ';'
    stringStream >> samplingPeriodMillis;

    PendulumLogEntry entry(currentTime, packetCount, bytesSentTotal, pendulumSensorValue,
                           samplingPeriodMillis, schedulingInfo);
    timepointLogs.emplace_back(entry);
}

void PendulumLogger::log(unsigned long long int packetCount, unsigned long long int bytesSentTotal, std::string payload){
    log(packetCount, bytesSentTotal, payload, nullptr);
}

nlohmann::json PendulumLogger::toJsonObject() {
    nlohmann::json jsonObject = {
            {"name",          name},
            {"timePointLogs", timepointLogs},
    };
    return jsonObject;
}

