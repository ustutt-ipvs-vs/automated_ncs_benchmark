//
// Created by david on 29.01.23.
//

#include "MultiPriorityTokenBucket.h"
#include <chrono>
#include "../Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h"

MultiPriorityTokenBucket::MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities,                                                  
                                                   std::vector<double> dataRateOfPriorities){
    this->b = b;
    this->r = r;
    currentBucketLevel = b;
    lastBucketFillTime = high_resolution_clock::now();
    packetCount = 0;
    currentSeverityLevel = 0;

    if (dataRateOfPriorities.size() != numPriorities) {
        throw std::runtime_error("Number of data rates must match numThresholds");
    }
    this->numPriorities = numPriorities;

    for(int i=0; i<numPriorities;i++){
        this->prioMapping.push_back(i);
    }

    calculateThresholdsAndCosts(r, dataRateOfPriorities);
}

MultiPriorityTokenBucket::MultiPriorityTokenBucket(double b, double r, unsigned int numThresholds, std::vector<double> thresholds,
                                                   std::vector<double> costs, std::vector<int> prioMapping) {
    this->b = b;
    this->r = r;
    currentBucketLevel = b;
    lastBucketFillTime = high_resolution_clock::now();
    packetCount = 0;
    currentSeverityLevel = 0;

    if (thresholds.size() != numThresholds || prioMapping.size() != numThresholds+1 || costs.size() != numThresholds+1) {
        throw std::runtime_error("numThresholds must match thresholds.size and prioMapping.size+1 and costs.size+1 (one extra prioMapping and cost value for BE)");
    }

    this->thresholdOfPriorities = thresholds;
    this->costOfPriorities = costs;
    this->prioMapping = prioMapping;
    this->numPriorities = numThresholds+1;
}

double MultiPriorityTokenBucket::getB() const {
    return b;
}

double MultiPriorityTokenBucket::getR() const {
    return r;
}

void MultiPriorityTokenBucket::reportPacketReadyToSend(int payloadSizeBytes) {
    fillBucket();
    int frameSizeBytes = toEthernetFrameSizeBytes(payloadSizeBytes);
    currentBucketLevel -= getCostOfCurrentPriority() * frameSizeBytes;   
    updateSeverityLevel();

    packetCount++;
}

void MultiPriorityTokenBucket::fillBucket() {
    uint64_t microsSinceLastBucketRefill = getMicrosSinceLastBucketFill();
    if (microsSinceLastBucketRefill > 100) {
        currentBucketLevel += r * 1.0E-6 * microsSinceLastBucketRefill;
        if (currentBucketLevel > b) {
            currentBucketLevel = b;
        }
        // reset Time Since Last Bucket Refill
        lastBucketFillTime = std::chrono::high_resolution_clock::now();
    }
}

int64_t MultiPriorityTokenBucket::getMicrosSinceLastBucketFill() {
    high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    return duration_cast<microseconds>(currentTime - lastBucketFillTime).count();
}


double MultiPriorityTokenBucket::getBucketLevel() {
    return currentBucketLevel;
}

unsigned int MultiPriorityTokenBucket::getPriority() {
    return prioMapping[currentSeverityLevel];
}

void MultiPriorityTokenBucket::updateSeverityLevel() {
    for(unsigned int severityLevel = 0; severityLevel < numPriorities-1; severityLevel++){
        if(currentBucketLevel >= thresholdOfPriorities[severityLevel]){
            currentSeverityLevel = severityLevel;
            return;
        }
    }
    currentSeverityLevel = numPriorities-1;
}

double MultiPriorityTokenBucket::getCostOfCurrentPriority() {
    return costOfPriorities.at(currentSeverityLevel);
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

SchedulingInfoEntry* MultiPriorityTokenBucket::getSchedulingInfoEntry() {
    return new TokenBucketInfoEntry(this);
}

std::string MultiPriorityTokenBucket::getDebugInfoString() {
    std::stringstream ss;
    ss << "Bucket Level: " << currentBucketLevel << " Priority: " << getPriority();
    return ss.str();
}

void MultiPriorityTokenBucket::resetState() {
    currentBucketLevel = b;
    lastBucketFillTime = high_resolution_clock::now();
    packetCount = 0;
    updateSeverityLevel();

}
