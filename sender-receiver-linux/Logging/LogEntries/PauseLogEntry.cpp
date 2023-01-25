//
// Created by david on 25.01.23.
//

#include "PauseLogEntry.h"

time_point<system_clock> PauseLogEntry::getTimePoint() const {
    return timePoint;
}

unsigned int PauseLogEntry::getPauseDurationMillis() const {
    return pauseDurationMillis;
}

PauseLogEntry::PauseLogEntry(const time_point<system_clock> &timePoint, unsigned int pauseDurationMillis) {
    this->timePoint = timePoint;
    this->pauseDurationMillis = pauseDurationMillis;
}


// JSON converter class (used implicitly by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const PauseLogEntry &entry) {
    unsigned long long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis", timePointAsUnixMillis},
            {"pauseDurationMillis",    entry.getPauseDurationMillis()},
    };
}
