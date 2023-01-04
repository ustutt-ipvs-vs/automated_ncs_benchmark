#ifndef UDP_SEND_TOKEN_BUCKET_CLION_STREAMLOGGER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_STREAMLOGGER_H


#include <string>
#include <vector>
#include "../TokenBucket.hpp"
#include "LogTimepointEntry.h"

using std::chrono::time_point;
using std::chrono::system_clock;

class StreamLogger {
public:
    StreamLogger(std::string name, double b, double r);
    void log(int packetCount, int bytesSentPerSecond, TokenBucket& tokenBucket);
    void saveToFile(std::string filename);
    void saveToFile();
    std::string toJsonString();
    nlohmann::json toJsonObject();

private:
    std::string name;
    std::time_t startTime;
    double b, r;
    std::vector<LogTimepointEntry> timepointLogs;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_STREAMLOGGER_H
