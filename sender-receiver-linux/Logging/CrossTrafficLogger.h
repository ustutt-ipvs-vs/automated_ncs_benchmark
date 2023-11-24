//
// Created by david on 10.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICLOGGER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICLOGGER_H


#include "Logger.h"

class CrossTrafficLogger : public Logger {
public:
    explicit CrossTrafficLogger(std::string name);
    void log(unsigned long long packetCount, unsigned long long bytesSentTotal, SchedulingInfoEntry* schedulingInfo);
    nlohmann::json toJsonObject() override;
    void reset() override;

private:
    std::vector<LogEntry> timepointLogs;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICLOGGER_H
