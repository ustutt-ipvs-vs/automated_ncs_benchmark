#include <fstream>
#include "StreamLogger.h"

StreamLogger::StreamLogger(std::string name, double b, double r) {
    this->name = name;
    this->b = b;
    this->r = r;
    this->startTime = std::time(0);
}

void StreamLogger::log(int packetCount, int bytesSentPerSecond, TokenBucket &tokenBucket) {
    time_point<system_clock> currentTime = system_clock::now();
    LogTimepointEntry entry(currentTime, tokenBucket.getPriority(), tokenBucket.getBucketLevel(), packetCount,
                            bytesSentPerSecond);
    timepointLogs.emplace_back(entry);
}

void StreamLogger::saveToFile(std::string filename) {
    std::fstream logfile;
    logfile.open(filename, std::ios_base::out);
    logfile << this->toJsonString();
    logfile.close();
}

void StreamLogger::saveToFile() {
    char dateStringBuffer[30];
    std::strftime(dateStringBuffer, 30, "%Y-%m-%d_%H-%M-%S", std::localtime(&startTime));
    std::string dateString(dateStringBuffer);
    std::string filename = name + "_" + dateString + ".json";
    saveToFile(filename);
}

std::string StreamLogger::toJsonString() {
    return toJsonObject().dump(4);
}

nlohmann::json StreamLogger::toJsonObject() {
    nlohmann::json jsonObject = {
            {"name",          name},
            {"b",             b},
            {"r",             r},
            {"timePointLogs", timepointLogs}
    };
    return jsonObject;
}
