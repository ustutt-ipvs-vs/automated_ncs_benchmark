#ifndef UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDER_H

#include <sockpp/udp_socket.h>
#include <sockpp/inet_address.h>
#include <atomic>
#include "../Scheduling/TokenBucket.hpp"
#include "../Logging/CrossTrafficLogger.h"

using sockpp::udp_socket;
using sockpp::inet_address;

class CrossTrafficSender {
private:
    int bytesPerSecond;
    int payloadSize = 1500 - 46; // 1500 is the MTU, 47 is the header size
    std::chrono::nanoseconds packetDelay;
    unsigned long long packetCount = 0;
    unsigned long long bytesSentTotal = 0;
    std::atomic<bool> stopSending{false};
    std::string name;
    CrossTrafficLogger logger;


    udp_socket senderSocket;
    inet_address receiverAddress;
    PriorityDeterminer *priorityDeterminer;
    std::vector<char> payload;

    void setupPayload();

    void sendPacket();


public:
    CrossTrafficSender(PriorityDeterminer* priorityDeterminer, int bytesPerSecond, std::string receiverHost,
                       int receiverPort, std::string name);

    void start();

    void stop();
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDER_H
