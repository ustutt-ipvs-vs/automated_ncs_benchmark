//
// Created by david on 24.01.23.
//

#ifndef SENDER_RECEIVER_LINUX_PRIORITYDETERMINER_H
#define SENDER_RECEIVER_LINUX_PRIORITYDETERMINER_H


#include "../Logging/LogEntries/SchedulingInfoEntries/SchedulingInfoEntry.h"

class PriorityDeterminer {
public:
    virtual int getPriority() = 0;
    virtual void reportPacketReadyToSend(int payloadSizeBytes) = 0;
    virtual SchedulingInfoEntry* getSchedulingInfoEntry() = 0;
    virtual std::string getDebugInfoString() = 0;
};


#endif //SENDER_RECEIVER_LINUX_PRIORITYDETERMINER_H
