//
// Created by david on 26.01.23.
//

#ifndef SENDER_RECEIVER_LINUX_SENDERFEEDBACKLOGENTRY_H
#define SENDER_RECEIVER_LINUX_SENDERFEEDBACKLOGENTRY_H

#include <chrono>
#include <string>
#include "../../nlohmann/json.hpp"

using std::chrono::time_point;
using std::chrono::system_clock;

class SenderFeedbackLogEntry {
private:
    time_point<system_clock> timePoint;
    unsigned int packetsLostTotal;
    unsigned int latencyMillis;
public:
    const time_point<system_clock> &getTimePoint() const;

    unsigned int getPacketsLostTotal() const;

    unsigned int getLatencyMillis() const;

public:
    SenderFeedbackLogEntry(const time_point<system_clock> &timePoint, unsigned int packetsLostTotal,
                           unsigned int latencyMillis);


};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const SenderFeedbackLogEntry &entry);


#endif //SENDER_RECEIVER_LINUX_SENDERFEEDBACKLOGENTRY_H
