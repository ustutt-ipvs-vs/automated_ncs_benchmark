//
// Created by david on 24.01.23.
//

#ifndef SENDER_RECEIVER_LINUX_CONSTANTPRIORITY_H
#define SENDER_RECEIVER_LINUX_CONSTANTPRIORITY_H


#include "PriorityDeterminer.h"

class ConstantPriority : public PriorityDeterminer {
private:
    int priority;

public:
    ConstantPriority(int priority);
    unsigned int getPriority() override;
    SchedulingInfoEntry * getSchedulingInfoEntry() override;
    void reportPacketReadyToSend(int payloadSizeBytes) override;
    std::string getDebugInfoString() override;
    void resetState() override;
};


#endif //SENDER_RECEIVER_LINUX_CONSTANTPRIORITY_H
