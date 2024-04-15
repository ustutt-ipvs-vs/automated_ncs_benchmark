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
    PendulumLogger* logger;
    std::string serialInputBuffer;
    int teensyHistorySize;
    int angleBias;
    std::vector<int> teensySamplingPeriods;
    float samplingPeriodSensitivityFactor;
    float samplingPeriodSensitivityOffset;

    uint64_t startTime;

    unsigned long long packetCount = 0;
    unsigned long long bytesSentTotal = 0;
    unsigned long long feedbackPacketsCount = 0;

    std::vector<int> networkDelaysPerPrio;

    std::function<void()> regularCallback;

public:
    PendulumSender(PriorityDeterminer* priorityDeterminer, std::string serialDeviceName, std::string receiverHost,
                   int receiverPort, int teensyHistorySize, std::vector<int> teensySamplingPeriods,
                   float samplingPeriodSensitivityFactor, float samplingPeriodSensitivityOffset,
                   std::function<void()> regularCallback, std::string logFilePrefix, int angleBias,
                   std::vector<int> networkDelaysPerPrio);
    void start();
    void stop();
    void swapPriorityDeterminer(PriorityDeterminer* newPriorityDeterminer, std::string logFilePrefix);
    void sendNewMptbConfigSignal(int number, std::string previousConfigName);
    void sendEndSignal(std::string previousConfigName);


private:
    void sendPacket(std::string payload);

    void handleSenderFeedback();
    uint64_t timeSinceEpochMillisec();
    std::string applyAngleBiasAndNetworkDelay(std::string payload, int networkDelay);
    void handleControlMessageFromTeensy();

    double calculatePoleAngle(const std::string &payload) const;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMSENDER_H
