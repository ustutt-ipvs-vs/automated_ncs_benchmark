//
// Created by david on 29.01.23.
//

#ifndef SENDER_RECEIVER_LINUX_MULTIPRIORITYTOKENBUCKET_H
#define SENDER_RECEIVER_LINUX_MULTIPRIORITYTOKENBUCKET_H


#include "TokenBucket.hpp"

class MultiPriorityTokenBucket : public TokenBucket {
public:
    MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities, unsigned int initialPriorityClass,
                             std::vector<double> dataRateOfPriorities);

    MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities, unsigned int initialPriorityClass,
                             std::vector<double> thresholds, std::vector<double> costs);

protected:
    void updatePriority() override;

    double getCostOfCurrentPriority() override;

private:
    void calculateThresholdsAndCosts(double r, std::vector<double> dataRateOfPriorities);

    std::vector<double> thresholdOfPriorities;
    std::vector<double> costOfPriorities;
    unsigned int numPriorities;
};


#endif //SENDER_RECEIVER_LINUX_MULTIPRIORITYTOKENBUCKET_H
