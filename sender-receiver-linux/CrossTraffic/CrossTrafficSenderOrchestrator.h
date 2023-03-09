#ifndef UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDERORCHESTRATOR_H
#define UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDERORCHESTRATOR_H


#include <thread>
#include <memory>
#include "CrossTrafficSender.h"

class CrossTrafficSenderOrchestrator {
private:
    std::vector<std::shared_ptr<CrossTrafficSender>> senders;
    std::vector<std::shared_ptr<std::thread>> senderThreads;

public:
    void initializeSendersByDefault();

    void initializeSendersWithParameters(std::string host, int port, std::vector<int> priorities,
                                         std::vector<int> dataRatesBytesPerSecond);

    void start();

    void stop();
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_CROSSTRAFFICSENDERORCHESTRATOR_H
