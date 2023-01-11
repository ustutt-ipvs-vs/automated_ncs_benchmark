//
// Created by david on 10.01.23.
//

#include "PendulumLogTimepointEntry.h"


int PendulumLogTimepointEntry::getAngleSensorValue() const {
    return angleSensorValue;
}

int PendulumLogTimepointEntry::getSamplingPeriodMillis() const {
    return samplingPeriodMillis;
}

PendulumLogTimepointEntry::PendulumLogTimepointEntry(time_point<system_clock> timePoint, int priority,
                                                     double bucketLevel, int packetCount, int bytesSentTotal,
                                                     int angleSensorValue, int samplingPeriodMillis)
        : LogTimepointEntry(timePoint, priority, bucketLevel, packetCount, bytesSentTotal) {
    this->angleSensorValue = angleSensorValue;
    this->samplingPeriodMillis = samplingPeriodMillis;
}

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const PendulumLogTimepointEntry &entry) {
    long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis",    timePointAsUnixMillis},
            {"priority",           entry.getPriority()},
            {"bucketLevel",        entry.getBucketLevel()},
            {"packetCount",        entry.getPacketCount()},
            {"bytesSentTotal", entry.getBytesSentTotal()},
            {"angleSensorValue", entry.getAngleSensorValue()},
            {"samplingPeriodMillis", entry.getSamplingPeriodMillis()}
    };
}