#ifndef SENDER_RECEIVER_LINUX_ACTUATORLOGENTRY_H
#define SENDER_RECEIVER_LINUX_ACTUATORLOGENTRY_H


#include <chrono>
#include <string>
#include "../../nlohmann/json.hpp"

using std::chrono::time_point;
using std::chrono::system_clock;

class ActuatorLogEntry {
private:
    time_point<system_clock> timePoint;
    unsigned int sampleNumber;
    float cartPosition, cartSpeed, poleAngle, xCart, vCart, xPole, vPole, uAcceleration, targetSpeed;

public:
    ActuatorLogEntry(const time_point<system_clock> &timePoint, unsigned int sampleNumber, float cartPosition,
                     float cartSpeed, float poleAngle, float xCart, float vCart, float xPole, float vPole,
                     float uAcceleration, float targetSpeed);

    time_point<system_clock> getTimePoint() const;

    unsigned int getSampleNumber() const;

    float getCartPosition() const;

    float getCartSpeed() const;

    float getPoleAngle() const;

    float getXCart() const;

    float getVCart() const;

    float getXPole() const;

    float getVPole() const;

    float getUAcceleration() const;

    float getTargetSpeed() const;
};

// JSON converter class (used implicitely by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const ActuatorLogEntry &entry);


#endif //SENDER_RECEIVER_LINUX_ACTUATORLOGENTRY_H
