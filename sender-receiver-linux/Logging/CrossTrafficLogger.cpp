//
// Created by david on 10.01.23.
//

#include "CrossTrafficLogger.h"

CrossTrafficLogger::CrossTrafficLogger(std::string name) : Logger(name) {}

void CrossTrafficLogger::log(unsigned long long packetCount, unsigned long long bytesSentTotal,
                             SchedulingInfoEntry *schedulingInfo) {
    time_point<system_clock> currentTime = system_clock::now();
    LogEntry entry(currentTime, packetCount, bytesSentTotal, schedulingInfo);
    timepointLogs.emplace_back(entry);
}

nlohmann::json CrossTrafficLogger::toJsonObject() {
    nlohmann::json jsonObject = {
            {"name",          name},
            {"timePointLogs", timepointLogs}
    };
    return jsonObject;
}
