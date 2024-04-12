#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGENTRY_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGENTRY_H


#include "LogEntry.h"

class PendulumLogEntry : public LogEntry {
public:
    PendulumLogEntry(time_point<system_clock> timePoint, unsigned long long packetCount,
                     unsigned long long bytesSentTotal, int angleSensorValue,
                     int samplingPeriodMillis, SchedulingInfoEntry *schedulingInfo);

    PendulumLogEntry(time_point<system_clock> timePoint, unsigned long long packetCount,
                     unsigned long long bytesSentTotal, int angleSensorValue,
                     int samplingPeriodMillis);

    int getAngleSensorValue() const;

    int getSamplingPeriodMillis() const;

private:
    int angleSensorValue;
    int samplingPeriodMillis;

};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const PendulumLogEntry &entry);

#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGENTRY_H
