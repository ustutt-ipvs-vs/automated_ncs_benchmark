//
// Created by david on 10.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGGER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGGER_H


#include "Logger.h"
#include "LogEntries/PendulumLogEntry.h"

class PendulumLogger : public Logger {
public:
    explicit PendulumLogger(std::string name);

    void log(unsigned long long packetCount, unsigned long long bytesSentTotal, std::string payload,
             SchedulingInfoEntry *schedulingInfo);
    void log(unsigned long long packetCount, unsigned long long bytesSentTotal, std::string payload);

    nlohmann::json toJsonObject() override;

private:
    std::vector<PendulumLogEntry> timepointLogs;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGGER_H
