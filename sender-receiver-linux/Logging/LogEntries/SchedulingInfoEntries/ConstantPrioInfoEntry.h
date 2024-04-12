#ifndef SENDER_RECEIVER_LINUX_CONSTANTPRIOINFOENTRY_H
#define SENDER_RECEIVER_LINUX_CONSTANTPRIOINFOENTRY_H


#include "SchedulingInfoEntry.h"
#include "../../../Scheduling/ConstantPriority.h"

class ConstantPrioInfoEntry : public SchedulingInfoEntry{
private:
    int priority;

public:
    ConstantPrioInfoEntry(int priority);
    ConstantPrioInfoEntry(ConstantPriority* constantPriority);
    int getPriority() const;

    nlohmann::json toJson() override;
};


#endif //SENDER_RECEIVER_LINUX_CONSTANTPRIOINFOENTRY_H
