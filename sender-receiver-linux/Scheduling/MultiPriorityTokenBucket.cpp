//
// Created by david on 29.01.23.
//

#include "MultiPriorityTokenBucket.h"

MultiPriorityTokenBucket::MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities,
                                                   unsigned int initialPriorityClass,
                                                   std::vector<double> dataRateOfPriorities)
        : TokenBucket(b, r, initialPriorityClass) {
    if (dataRateOfPriorities.size() != numPriorities) {
        throw std::runtime_error("Number of data rates must match numPriorities");
    }
    this->numPriorities = numPriorities;

    calculateThresholdsAndCosts(r, dataRateOfPriorities);
}

MultiPriorityTokenBucket::MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities,
                                                   unsigned int initialPriorityClass, std::vector<double> thresholds,
                                                   std::vector<double> costs)
        : TokenBucket(b, r, initialPriorityClass) {

    if (thresholds.size() != numPriorities || costs.size() != numPriorities) {
        throw std::runtime_error("Numbers of costs and thresholds must both match numPriorities");
    }

    this->thresholdOfPriorities = thresholds;
    this->costOfPriorities = costs;
    this->numPriorities = numPriorities;
}

void MultiPriorityTokenBucket::updatePriority() {
    for(unsigned int priority = 0; priority < numPriorities; priority++){
        if(currentBucketLevel >= thresholdOfPriorities[priority]){
            currentPriorityClass = priority;
            return;
        }
    }
}

double MultiPriorityTokenBucket::getCostOfCurrentPriority() {
    return costOfPriorities[currentPriorityClass];
}

/**
 * Converts the data rates into thresholds and costs. The data rates must be in bytes per second and should include not
 * only payload size but the total size of the ethernet frame.
 *
 * Note that the threshold values are chosen as fixed values 0, -100, -200, ...
 * This is done because the threshold values are unnecessary degrees of freedom as for any strictly monotonic
 * decreasing thresholds the costs can be chosen accordingly.
 *
 * @param r flow rate of the token bucket
 * @param dataRateOfPriorities data rates for each priority (best priority first) in bytes per second
 */
void MultiPriorityTokenBucket::calculateThresholdsAndCosts(double r, std::vector<double> dataRateOfPriorities) {
    double threshold = 0.0;
    for (double dataRate: dataRateOfPriorities) {
        thresholdOfPriorities.emplace_back(threshold);
        double cost = (r - threshold) / dataRate;
        costOfPriorities.emplace_back(cost);
        threshold -= 100.0;
    }
}
