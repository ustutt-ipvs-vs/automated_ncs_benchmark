#include "PendulumLogEntry.h"


int PendulumLogEntry::getAngleSensorValue() const {
    return angleSensorValue;
}

int PendulumLogEntry::getSamplingPeriodMillis() const {
    return samplingPeriodMillis;
}

PendulumLogEntry::PendulumLogEntry(time_point<system_clock> timePoint, unsigned long long packetCount,
                                   unsigned long long bytesSentTotal,
                                   int angleSensorValue, int samplingPeriodMillis)
        : LogEntry(timePoint, packetCount, bytesSentTotal) {
    this->angleSensorValue = angleSensorValue;
    this->samplingPeriodMillis = samplingPeriodMillis;
}

PendulumLogEntry::PendulumLogEntry(time_point<system_clock> timePoint, unsigned long long packetCount,
                                   unsigned long long bytesSentTotal,
                                   int angleSensorValue, int samplingPeriodMillis,
                                   SchedulingInfoEntry *schedulingInfo) : PendulumLogEntry(timePoint, packetCount,
                                                                                           bytesSentTotal,
                                                                                           angleSensorValue,
                                                                                           samplingPeriodMillis) {
    this->schedulingInfo = schedulingInfo;
}

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const PendulumLogEntry &entry) {
    unsigned long long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis",      timePointAsUnixMillis},
            {"packetCount",          entry.getPacketCount()},
            {"bytesSentTotal",       entry.getBytesSentTotal()},
            {"angleSensorValue",     entry.getAngleSensorValue()},
            {"samplingPeriodMillis", entry.getSamplingPeriodMillis()}
    };

    if (entry.getSchedulingInfo() != nullptr) {
        jsonObject["schedulingInfo"] = entry.getSchedulingInfo()->toJson();
    }
}
