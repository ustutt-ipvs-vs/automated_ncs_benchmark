#include "CrossTrafficSenderOrchestrator.h"
#include "../Scheduling/ConstantPriority.h"

void CrossTrafficSenderOrchestrator::initializeSendersByDefault() {
    std::string host = "10.0.1.2";

    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(0), 160'000, host, 3001, "Sender 0"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(1), 160'000, host, 3001, "Sender 1"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(2), 160'000, host, 3001, "Sender 2"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(3), 160'000, host, 3001, "Sender 3"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(4), 160'000, host, 3001, "Sender 4"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(5), 160'000, host, 3001, "Sender 5"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(6), 160'000, host, 3001, "Sender 6"));
    senders.emplace_back(
            std::make_shared<CrossTrafficSender>(new ConstantPriority(7), 160'000, host, 3001, "Sender 7"));
}

void
CrossTrafficSenderOrchestrator::initializeSendersWithParameters(std::string host, int port, std::vector<int> priorities,
                                                                std::vector<int> dataRatesBytesPerSecond) {
    if (priorities.size() != dataRatesBytesPerSecond.size()) {
        throw std::invalid_argument("The number of priorities and data rates must be the same");
    }

    for (int i = 0; i < priorities.size(); i++) {
        senders.emplace_back(
                std::make_shared<CrossTrafficSender>(new ConstantPriority(priorities[i]), dataRatesBytesPerSecond[i],
                                                     host, port, "Sender " + std::to_string(i)));
    }
}

void CrossTrafficSenderOrchestrator::start() {
    for (auto sender: senders) {
        auto senderThread = std::make_shared<std::thread>(&CrossTrafficSender::start, sender.get());
        senderThreads.emplace_back(senderThread);
    }
}

void CrossTrafficSenderOrchestrator::stop() {
    for (auto sender: senders) {
        sender->stop();
    }

    for (auto &senderThread: senderThreads) {
        senderThread->join();
    }
}
