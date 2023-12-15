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
    virtual ~Logger()= default; // virtual destructor to allow for polymorphic behavior
    virtual void setName(std::string name);
    virtual void saveToFile(std::string filename);
    virtual void saveToFile();
    virtual void saveTofileAsync(std::string filename);
    virtual void saveToFileAsync();
    virtual std::string toJsonString();
    virtual nlohmann::json toJsonObject() = 0;
    virtual void reset();

protected:
    std::string name;
    std::time_t startTime;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_LOGGER_H
