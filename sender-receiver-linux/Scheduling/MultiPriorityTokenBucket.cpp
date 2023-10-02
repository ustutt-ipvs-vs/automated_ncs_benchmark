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
    
    // Init with identity mapping:
    for(int i=0; i<numPriorities; i++){
        this->prioMapping.push_back(i);
    }

    calculateThresholdsAndCosts(r, dataRateOfPriorities);
}

MultiPriorityTokenBucket::MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities,
                                                   unsigned int initialPriorityClass, std::vector<double> thresholds,
                                                   std::vector<double> costs, std::vector<int> prioMapping)
        : TokenBucket(b, r, initialPriorityClass) {

    if (thresholds.size() != numPriorities || costs.size() != numPriorities) {
        throw std::runtime_error("Numbers of costs and thresholds must both match numPriorities");
    }

    this->thresholdOfPriorities = thresholds;
    this->costOfPriorities = costs;
    this->prioMapping = prioMapping;
    this->numPriorities = numPriorities;
}

unsigned int MultiPriorityTokenBucket::getPriority() {
    return prioMapping[currentPriorityClass];
}

void MultiPriorityTokenBucket::updatePriority() {
    for(unsigned int priority = 0; priority < numPriorities; priority++){
        if(currentBucketLevel >= thresholdOfPriorities[priority]){
            currentPriorityClass = std::max(priority, initialPriorityClass);
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
    for(double dataRate : dataRateOfPriorities){
        double cost = r / dataRate;
        costOfPriorities.emplace_back(cost);
    }

    // Best priority has zero threshold (as by MPTB specification)
    thresholdOfPriorities.emplace_back(0.0);

    // Choose thresholds such that the 'bucket' of each priority can send a b bytes burst before empty.
    for(int i = 1; i < numPriorities - 1; i++){
        double threshold = thresholdOfPriorities[i-1] - costOfPriorities[i] * b;
        thresholdOfPriorities.emplace_back(threshold);
    }

    // Worst priority has infinitely low threshold (best effort)
    thresholdOfPriorities.emplace_back(-std::numeric_limits<double>::infinity());
}
