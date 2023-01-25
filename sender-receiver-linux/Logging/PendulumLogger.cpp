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

void
PendulumLogger::log(unsigned long long int packetCount, unsigned long long int bytesSentTotal, std::string payload) {
    log(packetCount, bytesSentTotal, payload, nullptr);
}

void PendulumLogger::logActuator(std::string logString) {
    // The logString must be of the following form (without quotes):
    // 'logActuator:190;-0.001421;0.003402;-0.010472;-0.001421;0.003376;-0.010311;-0.027436;0.104382;0.013841'

    time_point<system_clock> currentTime = system_clock::now();

    unsigned int sampleNumber;
    float cartPosition, cartSpeed, poleAngle, xCart, vCart, xPole, vPole, uAcceleration, targetSpeed;

    // Parse logString to extract variables:
    std::stringstream stringStream(logString);
    stringStream.ignore(4); // skip 'logActuator:'
    stringStream >> sampleNumber;
    stringStream.ignore(); // skip ';'
    stringStream >> cartPosition;
    stringStream.ignore(); // skip ';'
    stringStream >> cartSpeed;
    stringStream.ignore(); // skip ';'
    stringStream >> poleAngle;
    stringStream.ignore(); // skip ';'
    stringStream >> xCart;
    stringStream.ignore(); // skip ';'
    stringStream >> vCart;
    stringStream.ignore(); // skip ';'
    stringStream >> xPole;
    stringStream.ignore(); // skip ';'
    stringStream >> vPole;
    stringStream.ignore(); // skip ';'
    stringStream >> uAcceleration;
    stringStream.ignore(); // skip ';'
    stringStream >> targetSpeed;

    ActuatorLogEntry entry(currentTime, sampleNumber, cartPosition, cartSpeed, poleAngle, xCart, vCart, xPole, vPole,
                           uAcceleration, targetSpeed);
    actuatorLogs.emplace_back(entry);
}

nlohmann::json PendulumLogger::toJsonObject() {
    nlohmann::json jsonObject = {
            {"name",          name},
            {"timePointLogs", timepointLogs},
            {"actuatorLogs", actuatorLogs},
            {"pauseLogs", pauseLogs}
    };
    return jsonObject;
}

void PendulumLogger::logPause(unsigned int durationMillis) {
    time_point<system_clock> currentTime = system_clock::now();
    PauseLogEntry entry(currentTime, durationMillis);
    pauseLogs.emplace_back(entry);
}


