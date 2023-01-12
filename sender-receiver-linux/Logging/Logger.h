#ifndef UDP_SEND_TOKEN_BUCKET_CLION_LOGGER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_LOGGER_H


#include <string>
#include <vector>
#include "../Scheduling/TokenBucket.hpp"
#include "LogEntries/LogEntry.h"

using std::chrono::time_point;
using std::chrono::system_clock;

class Logger {
public:
    Logger(std::string name);
    //void log(int packetCount, int bytesSentTotal, TokenBucket* tokenBucket);
    virtual void saveToFile(std::string filename);
    virtual void saveToFile();
    virtual std::string toJsonString();
    virtual nlohmann::json toJsonObject() = 0;

protected:
    std::string name;
    std::time_t startTime;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_LOGGER_H
