#include <iostream>
#include "TokenBucketPrioTest.hpp"

double TokenBucketPrioTest::getCostOfCurrentPriority() {
    return 1.0/(currentPriorityClass+1);
}

void TokenBucketPrioTest::updatePriority() {
    if(currentBucketLevel > b/2.0){
        currentPriorityClass = 0;
    } else if(currentBucketLevel <= b/2.0 && currentBucketLevel > 0.0){
        currentPriorityClass = 1;
    } else if(currentBucketLevel <= 0.0 && currentBucketLevel > -b/2.0){
        currentPriorityClass = 2;
    } else if(currentBucketLevel < -b/2.0){
        currentPriorityClass = 3;
    }
}

TokenBucketPrioTest::TokenBucketPrioTest(double b, double r, int initialPriorityClass) : TokenBucket(b, r, initialPriorityClass) {}

