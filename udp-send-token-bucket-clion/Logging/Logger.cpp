#include <fstream>
#include "Logger.h"

Logger::Logger(std::string name) {
    this->name = name;
    this->startTime = std::time(0);
}

void Logger::saveToFile(std::string filename) {
    std::fstream logfile;
    logfile.open(filename, std::ios_base::out);
    logfile << this->toJsonString();
    logfile.close();
}

void Logger::saveToFile() {
    char dateStringBuffer[30];
    std::strftime(dateStringBuffer, 30, "%Y-%m-%d_%H-%M-%S", std::localtime(&startTime));
    std::string dateString(dateStringBuffer);
    std::string filename = name + "_" + dateString + ".json";
    saveToFile(filename);
}

std::string Logger::toJsonString() {
    return toJsonObject().dump(4);
}

