#include "SenderFeedbackLogEntry.h"

const time_point<system_clock> &SenderFeedbackLogEntry::getTimePoint() const {
    return timePoint;
}

unsigned int SenderFeedbackLogEntry::getPacketsLostTotal() const {
    return packetsLostTotal;
}

unsigned int SenderFeedbackLogEntry::getLatencyMillis() const {
    return latencyMillis;
}

SenderFeedbackLogEntry::SenderFeedbackLogEntry(const time_point<system_clock> &timePoint, unsigned int packetsLostTotal,
                                               unsigned int latencyMillis) {
    this->timePoint = timePoint;
    this->packetsLostTotal = packetsLostTotal;
    this->latencyMillis = latencyMillis;
}

// JSON converter class (used implicitly by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const SenderFeedbackLogEntry &entry) {
    unsigned long long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis",  timePointAsUnixMillis},
            {"packetsLostTotal", entry.getPacketsLostTotal()},
            {"latencyMillis",    entry.getLatencyMillis()}
    };
}
