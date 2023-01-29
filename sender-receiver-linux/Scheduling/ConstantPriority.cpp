//
// Created by david on 24.01.23.
//

#include "ConstantPriority.h"
#include "../Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.h"

ConstantPriority::ConstantPriority(int priority) {
    this->priority = priority;
}

int ConstantPriority::getPriority() {
    return priority;
}

SchedulingInfoEntry *ConstantPriority::getSchedulingInfoEntry() {
    return new ConstantPrioInfoEntry(this);
}

void ConstantPriority::reportPacketReadyToSend(int payloadSizeBytes) {
    // Nothing to do here
}

std::string ConstantPriority::getDebugInfoString() {
    std::stringstream ss;
    ss << "Priority: " << priority;
    return ss.str();
}

void ConstantPriority::resetState() {
    // Nothing to do here
}
