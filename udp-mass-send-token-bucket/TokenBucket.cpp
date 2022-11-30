
#include "TokenBucket.hpp"


TokenBucket::TokenBucket(double b, double r, int initialPriorityClass){
    this->b = b;
    this->r = r;
    this->currentPriorityClass = initialPriorityClass;
    currentBucketLevel = b;
    lastBucketFillTime = high_resolution_clock::now();
}

int TokenBucket::getPriority(){
    return currentPriorityClass;
}

void TokenBucket::reportPacketReadyToSend(int payloadSizeBytes){
    currentBucketLevel -= getCostOfCurrentPriority() * payloadSizeBytes;
    fillBucket();
    updatePriority();

    packetCount++;
}

double TokenBucket::getCostOfCurrentPriority(){
    // TODO
    return 1.0;
}

void TokenBucket::fillBucket(){
    uint64_t microsSinceLastBucketRefill = getMicrosSinceLastBucketFill();
    if(microsSinceLastBucketRefill > 100){
        currentBucketLevel += r * microsSinceLastBucketRefill;
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

void TokenBucket::updatePriority(){
    // TODO
}

double TokenBucket::getBucketLevel(){
    return currentBucketLevel;
}
