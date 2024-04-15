#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMRECEIVER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMRECEIVER_H

#include <sockpp/udp_socket.h>
#include <CppLinuxSerial/SerialPort.hpp>
#include <atomic>
#include "../Logging/PendulumLogger.h"
#include "ReceiverConfig.h"
#include <chrono>

using sockpp::udp_socket;
using sockpp::inet_address;

using mn::CppLinuxSerial::SerialPort;
using mn::CppLinuxSerial::BaudRate;
using mn::CppLinuxSerial::NumDataBits;
using mn::CppLinuxSerial::Parity;
using mn::CppLinuxSerial::NumStopBits;
using sockpp::udp_socket;
using sockpp::inet_address;

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::time_point;
using std::chrono::system_clock;

class PendulumReceiver {
private:
    std::string serialDeviceName;
    inet_address receiverAddress;
    udp_socket receiverSocket;
    std::atomic<bool> stopReceiving{false};
    SerialPort serialActuator;
    PendulumLogger* logger;

    char receiveBuffer[1500];
    std::string networkInput;
    std::string serialInput;

    unsigned long long packetCount = 0;
    unsigned long long bytesReceivedTotal = 0;

    bool startedBalancing = false;

    int motorMaxRPM;
    double revolutionsPerTrack;
    ReceiverConfig::SwingUpBehavior swingUpBehavior;
    float swingUpDistanceFactor;
    float swingUpSpeedFactor;
    float swingUpAccelerationFactor;
    std::string kalmanAndControllerParamString;

public:
    PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort, int motorMaxRPM,
                     double revolutionsPerTrack, float swingUpDistanceFactor, float swingUpSpeedFactor,
                     float swingUpAccelerationFactor, ReceiverConfig::SwingUpBehavior swingUpBehavior,
                     std::string kalmanAndControllerParamString);
    void start();
    void stop();

    void startNewLogfile(std::string previousConfigName);

    void handleNewConfigSignal();

    void handleEndSignal();
};

#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMRECEIVER_H
