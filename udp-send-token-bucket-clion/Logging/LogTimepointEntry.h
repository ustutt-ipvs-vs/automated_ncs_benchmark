#ifndef UDP_SEND_TOKEN_BUCKET_CLION_LOGTIMEPOINTENTRY_H
#define UDP_SEND_TOKEN_BUCKET_CLION_LOGTIMEPOINTENTRY_H

#include <chrono>
#include "../nlohmann/json.hpp"

using std::chrono::time_point;
using std::chrono::system_clock;

class LogTimepointEntry {
private:
    time_point<system_clock> timePoint;
    int priority;
    double bucketLevel;
    int packetCount;
    int bytesSentPerSecond;
public:
    time_point<system_clock> getTimePoint() const;

    int getPriority() const;

    double getBucketLevel() const;

    int getPacketCount() const;

    int getBytesSentPerSecond() const;

public:
    LogTimepointEntry(time_point<system_clock> timePoint, int priority, double bucketLevel, int packetCount, int bytesSentPerSecond);
    nlohmann::json toJsonObject();
};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json& jsonObject, const LogTimepointEntry& entry);

#endif //UDP_SEND_TOKEN_BUCKET_CLION_LOGTIMEPOINTENTRY_H
