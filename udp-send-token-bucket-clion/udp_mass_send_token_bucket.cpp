#include <sockpp/udp_socket.h>
#include "TokenBucket.hpp"
#include "TokenBucketPrioTest.hpp"

using sockpp::udp_socket;
using sockpp::inet_address;

std::string receiverHost = "10.0.2.2";
int receiverPort = 3000;
int priority = 7;
int payloadSize = 1400;

int packetCount = 0;


int main(int argc, char* argv[]){
    if(argc >= 2){
        priority = std::stoi(argv[1]);
    }
    if(argc >= 3){
        receiverHost = argv[2];
    }
    if(argc >= 4){
        receiverPort = std::stoi(argv[3]);
    }

    TokenBucketPrioTest tokenBucketConstantPrio = TokenBucketPrioTest(2'000'000, 0.125, 0);
    TokenBucket &tokenBucket = tokenBucketConstantPrio;

    udp_socket senderSocket;
    inet_address receiverAddress(receiverHost, receiverPort);

    char payload[payloadSize];
    for(int i=0; i<payloadSize; i++){
        payload[i] = 'A';
    }

    while(true){
        if(tokenBucket.getPriority() > 1){
            tokenBucket.reportPacketReadyToSend(0);
            continue;
        }

        tokenBucket.reportPacketReadyToSend(payloadSize);
        priority = tokenBucket.getPriority();

        senderSocket.set_option(IPPROTO_IP, IP_TOS, priority);

        senderSocket.send_to(payload, receiverAddress);
        packetCount++;

        if(packetCount % 100 == 0){
            std::cout << "Sent " << packetCount << " packets. Bucket Level: " << tokenBucket.getBucketLevel()
            << " Prio: " << tokenBucket.getPriority() << std::endl;
        }
    }
}