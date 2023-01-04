#include "CrossTrafficSenderOrchestrator.h"

void CrossTrafficSenderOrchestrator::initializeSendersByDefault() {
    std::string host = "10.0.1.2";

    senders.emplace_back(std::make_shared<CrossTrafficSender>(2'000'000, 1'000'000, 5, 125'000, host, 3000, "Sender1"));
    senders.emplace_back(std::make_shared<CrossTrafficSender>(2'000'000, 1'000'000, 5, 125'000, host, 3001, "Sender2"));
    senders.emplace_back(std::make_shared<CrossTrafficSender>(2'000'000, 1'000'000, 5, 125'000, host, 3002, "Sender3"));
}

void CrossTrafficSenderOrchestrator::initializeSendersWithParameters() {
    // TODO
}

void CrossTrafficSenderOrchestrator::start() {
    for(auto sender : senders){
        auto senderThread = std::make_shared<std::thread>(&CrossTrafficSender::start, sender.get());
        senderThreads.emplace_back(senderThread);
    }
}

void CrossTrafficSenderOrchestrator::stop() {
    for (auto sender : senders) {
        sender->stop();
    }

    for(auto& senderThread : senderThreads){
        senderThread->join();
    }
}
