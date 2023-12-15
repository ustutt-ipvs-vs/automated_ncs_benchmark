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

    // Delete the logger object after saving to file to free up memory:
    delete this;
}

void Logger::saveToFile() {
    char dateStringBuffer[30];
    std::strftime(dateStringBuffer, 30, "%Y-%m-%d_%H-%M-%S", std::localtime(&startTime));
    std::string dateString(dateStringBuffer);
    std::string filename = name + "_" + dateString + ".json";
    saveToFile(filename);
}

void Logger::saveTofileAsync(std::string filename) {
    // Save to file in a separate thread
    std::thread saveThread([this, filename](){
        saveToFile(filename);
    });
    saveThread.detach();
}

void Logger::saveToFileAsync() {
    // Save to file in a separate thread
    std::thread saveThread([this](){
        saveToFile();
    });
    saveThread.detach();
}

std::string Logger::toJsonString() {
    return toJsonObject().dump(4);
}

void Logger::reset() {
    this->startTime = std::time(0);
}

void Logger::setName(std::string name) {
    this->name = name;
}

