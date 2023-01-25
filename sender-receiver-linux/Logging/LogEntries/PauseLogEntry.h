//
// Created by david on 19.01.23.
//

#ifndef SENDER_RECEIVER_LINUX_PAUSELOGENTRY_H
#define SENDER_RECEIVER_LINUX_PAUSELOGENTRY_H


#include <chrono>
#include <string>
#include "../../nlohmann/json.hpp"

using std::chrono::time_point;
using std::chrono::system_clock;

class PauseLogEntry {
private:
    time_point<system_clock> timePoint;
    unsigned int pauseDurationMillis;

public:
    PauseLogEntry(const time_point<system_clock> &timePoint, unsigned int pauseDurationMillis);

    time_point<system_clock> getTimePoint() const;

    unsigned int getPauseDurationMillis() const;

};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const PauseLogEntry &entry);


#endif //SENDER_RECEIVER_LINUX_PAUSELOGENTRY_H
