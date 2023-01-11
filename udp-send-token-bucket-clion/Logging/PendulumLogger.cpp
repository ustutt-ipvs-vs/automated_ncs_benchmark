//
// Created by david on 10.01.23.
//

#include "PendulumLogger.h"
#include <sstream>

PendulumLogger::PendulumLogger(std::string name, double b, double r) : Logger(name) {
    this->b = b;
    this->r = r;
}

void PendulumLogger::log(unsigned long long packetCount, unsigned long long bytesSentTotal, TokenBucket *tokenBucket, std::string payload) {
    time_point<system_clock> currentTime = system_clock::now();

    // payload must be of the form "1234;5678;\n"
    int pendulumSensorValue, samplingPeriodMillis;
    std::stringstream(payload) >> pendulumSensorValue >> samplingPeriodMillis;

    PendulumLogTimepointEntry entry(currentTime, tokenBucket->getPriority(), tokenBucket->getBucketLevel(), packetCount,
                                    bytesSentTotal, pendulumSensorValue, samplingPeriodMillis);
    timepointLogs.emplace_back(entry);
}

nlohmann::json PendulumLogger::toJsonObject() {
    nlohmann::json jsonObject = {
            {"name",          name},
            {"b",             b},
            {"r",             r},
            {"timePointLogs", timepointLogs}
    };
    return jsonObject;
}
