#ifndef UDP_SEND_TOKEN_BUCKET_CLION_LOGENTRY_H
#define UDP_SEND_TOKEN_BUCKET_CLION_LOGENTRY_H

#include <chrono>
#include "../../nlohmann/json.hpp"
#include "SchedulingInfoEntries/SchedulingInfoEntry.h"

using std::chrono::time_point;
using std::chrono::system_clock;

class LogEntry {
protected:
    time_point<system_clock> timePoint;
    SchedulingInfoEntry *schedulingInfo = nullptr;
    unsigned long long packetCount;
    unsigned long long bytesSentTotal;
public:
    time_point<system_clock> getTimePoint() const;

    unsigned long long getPacketCount() const;

    unsigned long long getBytesSentTotal() const;

    SchedulingInfoEntry *getSchedulingInfo() const;

public:
    LogEntry(time_point<system_clock> timePoint, unsigned long long packetCount, unsigned long long bytesSentTotal,
             SchedulingInfoEntry *schedulingInfo);

    LogEntry(time_point<system_clock> timePoint, unsigned long long packetCount, unsigned long long bytesSentTotal);

    nlohmann::json toJsonObject();
};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const LogEntry &entry);

#endif //UDP_SEND_TOKEN_BUCKET_CLION_LOGENTRY_H
