//
// Created by david on 19.01.23.
//

#include <sstream>
#include "ActuatorLogEntry.h"

time_point<system_clock> ActuatorLogEntry::getTimePoint() const {
    return timePoint;
}

unsigned int ActuatorLogEntry::getSampleNumber() const {
    return sampleNumber;
}

float ActuatorLogEntry::getCartPosition() const {
    return cartPosition;
}

float ActuatorLogEntry::getCartSpeed() const {
    return cartSpeed;
}

float ActuatorLogEntry::getPoleAngle() const {
    return poleAngle;
}

float ActuatorLogEntry::getXCart() const {
    return xCart;
}

float ActuatorLogEntry::getVCart() const {
    return vCart;
}

float ActuatorLogEntry::getXPole() const {
    return xPole;
}

float ActuatorLogEntry::getVPole() const {
    return vPole;
}

float ActuatorLogEntry::getUAcceleration() const {
    return uAcceleration;
}

float ActuatorLogEntry::getTargetSpeed() const {
    return targetSpeed;
}

ActuatorLogEntry::ActuatorLogEntry(const time_point<system_clock> &timePoint, unsigned int sampleNumber, float cartPosition,
                                   float cartSpeed, float poleAngle, float xCart, float vCart, float xPole, float vPole,
                                   float uAcceleration, float targetSpeed) : timePoint(timePoint), sampleNumber(sampleNumber),
                                                                       cartPosition(cartPosition), cartSpeed(cartSpeed),
                                                                       poleAngle(poleAngle), xCart(xCart), vCart(vCart),
                                                                       xPole(xPole), vPole(vPole),
                                                                       uAcceleration(uAcceleration),
                                                                       targetSpeed(targetSpeed) {}


// JSON converter class (used implicitly by nlohmann/json.hpp):
void to_json(nlohmann::json &jsonObject, const ActuatorLogEntry &entry) {
    long timePointAsUnixMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
            entry.getTimePoint().time_since_epoch()).count();

    jsonObject = {
            {"timePointMillis", timePointAsUnixMillis},
            {"sampleNumber",    entry.getSampleNumber()},
            {"cartPosition",    entry.getCartPosition()},
            {"cartSpeed",       entry.getCartSpeed()},
            {"poleAngle",       entry.getPoleAngle()},
            {"xCart",           entry.getXCart()},
            {"vCart",           entry.getVCart()},
            {"xPole",           entry.getXPole()},
            {"vPole",           entry.getVPole()},
            {"uAcceleration",   entry.getUAcceleration()},
            {"targetSpeed",     entry.getTargetSpeed()}
    };
}
