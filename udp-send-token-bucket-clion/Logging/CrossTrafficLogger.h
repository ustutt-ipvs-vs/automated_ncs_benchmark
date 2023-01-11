//
// Created by david on 10.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICLOGGER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICLOGGER_H


#include "Logger.h"

class CrossTrafficLogger : public Logger {
public:
    CrossTrafficLogger(std::string name, double b, double r);
    void log(unsigned long long packetCount, unsigned long long bytesSentTotal, TokenBucket* tokenBucket);
    nlohmann::json toJsonObject() override;

private:
    double b, r;
    std::vector<LogTimepointEntry> timepointLogs;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICLOGGER_H
