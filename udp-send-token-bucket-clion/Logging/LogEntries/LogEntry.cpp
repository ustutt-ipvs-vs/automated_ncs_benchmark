#include "LogEntry.h"

LogEntry::LogEntry(time_point<system_clock> timePoint,
                   unsigned long long packetCount, unsigned long long bytesSentTotal) {
    this->timePoint = timePoint;
    this->packetCount = packetCount;
    this->bytesSentTotal = bytesSentTotal;
}

LogEntry::LogEntry(time_point<system_clock> timePoint, unsigned long long packetCount,
                   unsigned long long bytesSentTotal,
                   SchedulingInfoEntry *schedulingInfo) : LogEntry(timePoint, packetCount, bytesSentTotal) {
    this->schedulingInfo = schedulingInfo;
}

time_point<system_clock> LogEntry::getTimePoint() const {
    return timePoint;
}

unsigned long long LogEntry::getPacketCount() const {
    return packetCount;
}

unsigned long long LogEntry::getBytesSentTotal() const {
    return bytesSentTotal;
}

nlohmann::json LogEntry::toJsonObject() {
    nlohmann::json j = *this; // uses to_json(nlohmann::json& jsonObject, const LogEntry& entry)
    return j;
}

SchedulingInfoEntry *LogEntry::getSchedulingInfo() const {
    return schedulingInfo;
}

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const LogEntry &entry) {
    long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis", timePointAsUnixMillis},
            {"packetCount",     entry.getPacketCount()},
            {"bytesSentTotal",  entry.getBytesSentTotal()}
    };

    if (entry.getSchedulingInfo() != nullptr) {
        jsonObject["schedulingInfo"] = entry.getSchedulingInfo()->toJson();
    }
}

