#include "LogTimepointEntry.h"

LogTimepointEntry::LogTimepointEntry(time_point<system_clock> timePoint, int priority, double bucketLevel,
                                     int packetCount, int bytesSentTotal) {
    this->timePoint = timePoint;
    this->priority = priority;
    this->bucketLevel = bucketLevel;
    this->packetCount = packetCount;
    this->bytesSentTotal = bytesSentTotal;
}

time_point<system_clock> LogTimepointEntry::getTimePoint() const {
    return timePoint;
}

int LogTimepointEntry::getPriority() const {
    return priority;
}

double LogTimepointEntry::getBucketLevel() const {
    return bucketLevel;
}

int LogTimepointEntry::getPacketCount() const {
    return packetCount;
}

int LogTimepointEntry::getBytesSentTotal() const {
    return bytesSentTotal;
}

nlohmann::json LogTimepointEntry::toJsonObject() {
    nlohmann::json j = *this; // uses to_json(nlohmann::json& jsonObject, const LogTimepointEntry& entry)
    return j;
}

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const LogTimepointEntry &entry) {
    long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis",    timePointAsUnixMillis},
            {"priority",           entry.getPriority()},
            {"bucketLevel",        entry.getBucketLevel()},
            {"packetCount",        entry.getPacketCount()},
            {"bytesSentTotal", entry.getBytesSentTotal()}
    };
}

