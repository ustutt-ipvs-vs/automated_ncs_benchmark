#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMSENDER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMSENDER_H


#include <string>
#include <ctime>
#include <sockpp/udp_socket.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include <atomic>
#include "../Logging/PendulumLogger.h"

using sockpp::udp_socket;
using sockpp::inet_address;

using mn::CppLinuxSerial::SerialPort;
using mn::CppLinuxSerial::BaudRate;
using mn::CppLinuxSerial::NumDataBits;
using mn::CppLinuxSerial::Parity;
using mn::CppLinuxSerial::NumStopBits;
using sockpp::udp_socket;
using sockpp::inet_address;

class PendulumSender {
private:
    std::string serialDeviceName;
    inet_address receiverAddress;
    udp_socket senderSocket;
    PriorityDeterminer* priorityDeterminer;
    std::atomic<bool> stopSending{false};
    SerialPort serialSensor;
    PendulumLogger logger;
    std::string serialInputBuffer;
    bool pendulumStarted = false;
    int teensyHistorySize;
    std::vector<int> teensySamplingPeriods;

    uint64_t startTime;
    uint64_t lastLogTime = 0;

    double currentRunAVG = 0;
    int currentRunValues = 0;
    int currentRunNr = 1;

    double allRunsAVG = 0;
    int allRuns = 0; 

    unsigned long long packetCount = 0;
    unsigned long long bytesSentTotal = 0;
    unsigned long long feedbackPacketsCount = 0;

    std::function<void()> regularCallback;

public:
    PendulumSender(PriorityDeterminer* priorityDeterminer, std::string serialDeviceName, std::string receiverHost,
                   int receiverPort, int teensyHistorySize, std::vector<int> teensySamplingPeriods,
                   std::function<void()> regularCallback = nullptr, std::string logFilePrefix = "pendulumsender");
    void start();
    void stop();
    void swapPriorityDeterminer(PriorityDeterminer* newPriorityDeterminer, std::string logFilePrefix);


private:
    void sendPacket(std::string payload);

    void handleSenderFeedback();
    uint64_t timeSinceEpochMillisec();
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMSENDER_H
