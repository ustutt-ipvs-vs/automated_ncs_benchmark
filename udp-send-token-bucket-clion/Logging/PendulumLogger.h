//
// Created by david on 10.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGGER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGGER_H


#include "Logger.h"
#include "PendulumLogTimepointEntry.h"

class PendulumLogger : public Logger{
public:
    PendulumLogger(std::string name, double b, double r);
    void log(unsigned long long packetCount, unsigned long long bytesSentTotal, TokenBucket* tokenBucket, std::string payload);
    nlohmann::json toJsonObject() override;

private:
    double b, r;
    std::vector<PendulumLogTimepointEntry> timepointLogs;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMLOGGER_H
