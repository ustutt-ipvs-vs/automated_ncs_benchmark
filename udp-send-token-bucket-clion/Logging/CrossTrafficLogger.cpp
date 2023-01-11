//
// Created by david on 10.01.23.
//

#include "CrossTrafficLogger.h"

CrossTrafficLogger::CrossTrafficLogger(std::string name, double b, double r) : Logger(name) {
    this->b = b;
    this->r = r;
}

void CrossTrafficLogger::log(unsigned long long packetCount, unsigned long long bytesSentTotal, TokenBucket* tokenBucket) {
    time_point<system_clock> currentTime = system_clock::now();
    LogTimepointEntry entry(currentTime, tokenBucket->getPriority(), tokenBucket->getBucketLevel(), packetCount,
                            bytesSentTotal);
    timepointLogs.emplace_back(entry);
}

nlohmann::json CrossTrafficLogger::toJsonObject() {
    nlohmann::json jsonObject = {
            {"name",          name},
            {"b",             b},
            {"r",             r},
            {"timePointLogs", timepointLogs}
    };
    return jsonObject;
}
