#include <vector>
#include "TeensyPortDetector.h"
#include <filesystem>

/**
 * Returns the name of the first /dev/ttyACM* device that has "ID_VENDOR=Teensyduino" in its udevadm info.
 */
std::string TeensyPortDetector::findTeensySerialDevice() {
    // First get a list of all /dev/ttyACM* devices:
    std::vector<std::string> serialDevices = getAllSerialDeviceNames();
    if(serialDevices.empty()){
        throw std::runtime_error("No /dev/ttyACM* devices found");
    }

    // Now scan all of them for "ID_VENDOR=Teensyduino" and return the first one that matches:
    for(const auto& deviceName : serialDevices){
        std::string command = "udevadm info --query=property " + deviceName + " | grep ID_VENDOR=";
        std::string result = runShellCommand(command);
        if(result.find("ID_VENDOR=Teensyduino") != std::string::npos){
            return deviceName;
        }
    }
    throw std::runtime_error("No Teensy device found or udevadm not installed");
}

/**
 * Returns a list of all /dev/ttyACM* devices.
 */
std::vector<std::string> TeensyPortDetector::getAllSerialDeviceNames() {
    std::vector<std::string> matchingFiles;
    for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
        if (entry.is_character_file()) {
            std::string filePath = entry.path().string();
            if (filePath.find("/dev/ttyACM") != std::string::npos) {
                matchingFiles.push_back(filePath);
            }
        }
    }
    return matchingFiles;
}


std::string TeensyPortDetector::runShellCommand(const std::string& command) {
    // Open a pipe to the shell command
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return "Error: Unable to run the command.";
    }

    // Read the command's output into a string
    std::string result;
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    // Close the pipe
    pclose(pipe);

    return result;
}