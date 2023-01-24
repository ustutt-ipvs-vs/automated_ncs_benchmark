#include <thread>
#include "CrossTrafficSender.h"
#include "../Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h"

CrossTrafficSender::CrossTrafficSender(PriorityDeterminer* priorityDeterminer, int bytesPerSecond,
                                       std::string receiverHost, int receiverPort, std::string name): logger(name) {
    this->bytesPerSecond = bytesPerSecond;
    this->name = name;
    receiverAddress = inet_address(receiverHost, receiverPort);

    int packetDelayNanos = (1.0 / (static_cast<double>(bytesPerSecond) / static_cast<double>(payloadSize))) * 1E9;
    packetDelay = std::chrono::nanoseconds(packetDelayNanos);

    this->priorityDeterminer = priorityDeterminer;

    setupPayload();

}

void CrossTrafficSender::start() {
    while (!stopSending) {
        sendPacket();
        std::this_thread::sleep_for(packetDelay);
    }
}

void CrossTrafficSender::stop() {
    stopSending = true;
    logger.saveToFile();
}

void CrossTrafficSender::setupPayload() {
    payload = std::vector<char>(payloadSize);
    for (int i = 0; i < payloadSize; i++) {
        payload[i] = 'A';
    }
}

void CrossTrafficSender::sendPacket() {
    priorityDeterminer->reportPacketReadyToSend(payloadSize);
    int priority = priorityDeterminer->getPriority();
    senderSocket.set_option(SOL_SOCKET, SO_PRIORITY, priority);
    senderSocket.send_to(payload.data(), receiverAddress);
    packetCount++;
    bytesSentTotal += payloadSize;

    if (packetCount % 100 == 0) {
        logger.log(packetCount, bytesSentTotal, priorityDeterminer->getSchedulingInfoEntry());
        std::cout << name << ": "
                  << "Sent " << packetCount << " packets. "
                  << priorityDeterminer->getDebugInfoString()
                  << std::endl;
    }
}
