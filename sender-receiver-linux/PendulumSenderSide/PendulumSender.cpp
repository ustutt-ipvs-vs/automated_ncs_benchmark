//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"
#include "../Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h"

PendulumSender::PendulumSender(PriorityDeterminer* priorityDeterminer, std::string serialDeviceName, std::string receiverHost, int receiverPort) : logger("pendulumsender") {
    this->serialDeviceName = serialDeviceName;
    receiverAddress = inet_address(receiverHost, receiverPort);

    serialSensor = SerialPort(serialDeviceName, BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialSensor.SetTimeout(-1);

    this->priorityDeterminer = priorityDeterminer;
}

void PendulumSender::start() {
    serialSensor.Open();

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
    priorityDeterminer->reportPacketReadyToSend(payload.size());
    int priority = priorityDeterminer->getPriority();
    senderSocket.set_option(SOL_SOCKET, SO_PRIORITY, priority);
    senderSocket.send_to(payload, receiverAddress);
    packetCount++;
    bytesSentTotal += payload.size();

    logger.log(packetCount, bytesSentTotal, payload, priorityDeterminer->getSchedulingInfoEntry());

    if (packetCount % 10 == 0) {
        std::cout << "PendulumSender: "
                  << "Sent " << packetCount << " packets. "
                  << priorityDeterminer->getDebugInfoString()
                  << " Payload: " << payload
                  << std::endl;
    }
}
