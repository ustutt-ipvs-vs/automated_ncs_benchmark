//
// Created by david on 10.01.23.
//

#include "PendulumLogger.h"
#include "LogEntries/SenderFeedbackLogEntry.h"
#include <sstream>

PendulumLogger::PendulumLogger(std::string name) : Logger(name) {}

void PendulumLogger::log(unsigned long long packetCount, unsigned long long bytesSentTotal, std::string payload,
                         SchedulingInfoEntry *schedulingInfo) {
    time_point<system_clock> currentTime = system_clock::now();

    // The sample value string is of the following form:
    // S:encoderValue;samplingPeriodMillis;sequenceNumber;currentTime;\n
    // for example
    // S:-1204;50;1234;62345234;\n
    int pendulumSensorValue, samplingPeriodMillis;
    std::stringstream stringStream(payload);
    stringStream.ignore(2); // skip 'S:'
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
    // 'log:190;-0.001421;0.003402;-0.010472;-0.001421;0.003376;-0.010311;-0.027436;0.104382;0.013841'

    time_point<system_clock> currentTime = system_clock::now();

    unsigned int sampleNumber;
    float cartPosition, cartSpeed, poleAngle, xCart, vCart, xPole, vPole, uAcceleration, targetSpeed;

    // Parse logString to extract variables:
    std::stringstream stringStream(logString);
    stringStream.ignore(4); // skip 'log:'
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
            {"pauseLogs", pauseLogs},
            {"senderFeedbackLogs", senderFeedbackLogs}
    };
    return jsonObject;
}

void PendulumLogger::logPause(unsigned int durationMillis) {
    time_point<system_clock> currentTime = system_clock::now();
    PauseLogEntry entry(currentTime, durationMillis);
    pauseLogs.emplace_back(entry);
}

void PendulumLogger::logSenderFeedback(std::string senderFeedbackString) {
    // The logging information sent to the sender site Linux client has the following form:
    // packetsLostTotal;latencyMillis;\n
    // for example
    // FB:55;13;\n

    time_point<system_clock> currentTime = system_clock::now();

    unsigned int packetsLostTotal, latencyMillis;

    std::stringstream stringStream(senderFeedbackString);
    stringStream.ignore(3); // skip 'FB:'
    stringStream >> packetsLostTotal;
    stringStream.ignore(); // skip ';'
    stringStream >> latencyMillis;

    SenderFeedbackLogEntry entry(currentTime, packetsLostTotal, latencyMillis);
    senderFeedbackLogs.emplace_back(entry);
}

void PendulumLogger::reset() {
    Logger::reset();
    timepointLogs.clear();
    actuatorLogs.clear();
    pauseLogs.clear();
    senderFeedbackLogs.clear();
}


