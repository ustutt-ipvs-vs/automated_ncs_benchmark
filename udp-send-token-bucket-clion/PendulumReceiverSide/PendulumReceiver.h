//
// Created by david on 11.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMRECEIVER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMRECEIVER_H

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

class PendulumReceiver {
private:
    std::string serialDeviceName;
    inet_address receiverAddress;
    udp_socket receiverSocket;
    std::atomic<bool> stopReceiving{false};
    SerialPort serialActuator;
    PendulumLogger logger;

    char receiveBuffer[1500];
    std::string networkInput;
    std::string serialInput;

    unsigned long long packetCount = 0;
    unsigned long long bytesReceivedTotal = 0;

public:
    PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort);
    void start();
    void stop();

};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_PENDULUMRECEIVER_H
