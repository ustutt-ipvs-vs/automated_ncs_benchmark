#include <fstream>
#include <iostream>
#include <thread>
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
    std::cout << "Log Saved log to " << filename << std::endl;
    delete this;
}

void Logger::saveToFile() {
    char dateStringBuffer[30];
    std::strftime(dateStringBuffer, 30, "%Y-%m-%d_%H-%M-%S", std::localtime(&startTime));
    std::string dateString(dateStringBuffer);
    std::string filename = name + "_" + dateString + ".json";
    saveToFile(filename);
    delete this;
}

void Logger::saveTofileAsync(std::string filename) {
    // Save to file in a separate thread
    std::thread saveThread([this, filename](){
        saveToFile(filename);
        delete this;
    });
    saveThread.detach();
}

void Logger::saveToFileAsync() {
    // Save to file in a separate thread
    std::thread saveThread([this](){
        saveToFile();
        delete this;
    });
    saveThread.detach();
}

std::string Logger::toJsonString() {
    return toJsonObject().dump(4);
}

