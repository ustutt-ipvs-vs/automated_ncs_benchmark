//
// Created by david on 24.01.23.
//

#include "ConstantPrioInfoEntry.h"

int ConstantPrioInfoEntry::getPriority() const {
    return priority;
}

nlohmann::json ConstantPrioInfoEntry::toJson() {
    nlohmann::json jsonObject = {
            {"priority", priority},
    };
    return jsonObject;
}

ConstantPrioInfoEntry::ConstantPrioInfoEntry(int priority) {
    this->priority = priority;
}

ConstantPrioInfoEntry::ConstantPrioInfoEntry(ConstantPriority *constantPriority) {
    this->priority = constantPriority->getPriority();
}
