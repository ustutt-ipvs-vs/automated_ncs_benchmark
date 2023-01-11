//
// Created by david on 10.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGTIMEPOINTENTRY_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGTIMEPOINTENTRY_H


#include "LogTimepointEntry.h"

class PendulumLogTimepointEntry : public LogTimepointEntry {
public:
    PendulumLogTimepointEntry(time_point<system_clock> timePoint, int priority, double bucketLevel,
                              int packetCount, int bytesSentTotal, int angleSensorValue, int samplingPeriodMillis);
    int getAngleSensorValue() const;
    int getSamplingPeriodMillis() const;

private:
    int angleSensorValue;
    int samplingPeriodMillis;

};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const PendulumLogTimepointEntry &entry);

#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGTIMEPOINTENTRY_H
