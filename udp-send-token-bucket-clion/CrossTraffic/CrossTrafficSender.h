#ifndef UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDER_H
#define UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDER_H

#include <sockpp/udp_socket.h>
#include <sockpp/inet_address.h>
#include <atomic>
#include "../TokenBucket.hpp"
#include "../TokenBucketPrioTest.hpp"
#include "../Logging/StreamLogger.h"

using sockpp::udp_socket;
using sockpp::inet_address;

class CrossTrafficSender {
private:
    int bytesPerSecond;
    int payloadSize = 1400;
    std::chrono::nanoseconds packetDelay;
    int packetCount = 0;
    std::atomic<bool> stopSending{false};
    std::string name;
    StreamLogger logger;


    udp_socket senderSocket;
    inet_address receiverAddress;
    TokenBucketPrioTest tokenBucket;
    std::vector<char> payload;
    void setupPayload();
    void sendPacket();


public:
    CrossTrafficSender(double b, double r, int initialPriority, int bytesPerSecond, std::string receiverHost, int receiverPort, std::string name);
    void start();
    void stop();
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDER_H
