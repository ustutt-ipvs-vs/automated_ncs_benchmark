//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"
#include "../Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h"

PendulumSender::PendulumSender(std::string serialDeviceName, std::string receiverHost, int receiverPort, double b,
                               double r, int initialPriority) : logger("pendulumsender") {
    this->serialDeviceName = serialDeviceName;
    receiverAddress = inet_address(receiverHost, receiverPort);

    serialSensor = SerialPort(serialDeviceName, BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialSensor.SetTimeout(-1);
    serialSensor.Open();

    tokenBucket = new TokenBucketPrioTest(b, r, initialPriority);
}

void PendulumSender::start() {
    //serialSensor.Open();
    // Wait for first serial values to arrive, before going into main loop:
    std::cout << "Waiting for first sensor value" << std::endl;
    serialSensor.Read(serialInputBuffer);
    std::cout << serialInputBuffer << std::endl;

    while (!stopSending) {
        serialSensor.Read(serialInputBuffer);
        sendPacket(serialInputBuffer);
    }
}

void PendulumSender::stop() {
    stopSending = true;
    serialSensor.Close();
    logger.saveToFile();
}

void PendulumSender::sendPacket(std::string payload) {
    tokenBucket->reportPacketReadyToSend(payload.size());
    int priority = tokenBucket->getPriority();
    senderSocket.set_option(SOL_SOCKET, SO_PRIORITY, priority);
    senderSocket.send_to(payload, receiverAddress);
    packetCount++;
    bytesSentTotal += payload.size();

    logger.log(packetCount, bytesSentTotal, payload, new TokenBucketInfoEntry(tokenBucket));

    if (packetCount % 10 == 0) {
        std::cout << "PendulumSender: "<< ": "
                  << "Sent " << packetCount << " packets."
                  << " Bucket Level: " << tokenBucket->getBucketLevel()
                  << " Prio: " << tokenBucket->getPriority()
                  << " Payload: " << payload
                  << std::endl;
    }
}
