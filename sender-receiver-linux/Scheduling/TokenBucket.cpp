
#include "TokenBucket.hpp"
#include "../Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h"

TokenBucket::TokenBucket(double b, double r, int initialPriorityClass){
    this->b = b;
    this->r = r;
    this->currentPriorityClass = initialPriorityClass;
    currentBucketLevel = b;
    lastBucketFillTime = high_resolution_clock::now();
    packetCount = 0;
}

int TokenBucket::getPriority(){
    return currentPriorityClass;
}

void TokenBucket::reportPacketReadyToSend(int payloadSizeBytes){
    int frameSizeBytes = toEthernetFrameSizeBytes(payloadSizeBytes);
    currentBucketLevel -= getCostOfCurrentPriority() * frameSizeBytes;
    fillBucket();
    updatePriority();

    packetCount++;
}

void TokenBucket::fillBucket(){
    uint64_t microsSinceLastBucketRefill = getMicrosSinceLastBucketFill();
    if(microsSinceLastBucketRefill > 100){
        currentBucketLevel += r * 1.0E-6 * microsSinceLastBucketRefill;
        if(currentBucketLevel > b){
            currentBucketLevel = b;
        }
        resetTimeSinceLastBucketRefill();
    }
}

int64_t TokenBucket::getMicrosSinceLastBucketFill(){
    high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    return duration_cast<microseconds>(currentTime - lastBucketFillTime).count();
}

void TokenBucket::resetTimeSinceLastBucketRefill(){
    lastBucketFillTime = std::chrono::high_resolution_clock::now();
}

double TokenBucket::getBucketLevel(){
    return currentBucketLevel;
}

double TokenBucket::getB() const {
    return b;
}

double TokenBucket::getR() const {
    return r;
}

SchedulingInfoEntry *TokenBucket::getSchedulingInfoEntry() {
    return new TokenBucketInfoEntry(this);
}

std::string TokenBucket::getDebugInfoString() {
    std::stringstream ss;
    ss << "Bucket Level: " << currentBucketLevel << " Priority: " << currentPriorityClass;
    return ss.str();
}
