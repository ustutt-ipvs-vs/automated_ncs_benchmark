//
// Created by david on 11.01.23.
//

#include "TokenBucketInfoEntry.h"

double TokenBucketInfoEntry::getB() const {
    return b;
}

double TokenBucketInfoEntry::getR() const {
    return r;
}

int TokenBucketInfoEntry::getPriority() const {
    return priority;
}

double TokenBucketInfoEntry::getBucketLevel() const {
    return bucketLevel;
}

nlohmann::json TokenBucketInfoEntry::toJson() {
    nlohmann::json jsonObject = {
            {"b", b},
            {"r", r},
            {"priority", priority},
            {"bucketLevel", bucketLevel}
    };
    return jsonObject;
}

TokenBucketInfoEntry::TokenBucketInfoEntry(TokenBucket *tokenBucket) {
    b =tokenBucket->getB();
    r = tokenBucket->getR();
    priority = tokenBucket->getPriority();
    bucketLevel = tokenBucket->getBucketLevel();
}

TokenBucketInfoEntry::TokenBucketInfoEntry(MultiPriorityTokenBucket* tokenBucket) {
    b = tokenBucket->getB();
    r = tokenBucket->getR();
    priority = tokenBucket->getPriority();
    bucketLevel = tokenBucket->getBucketLevel();
}
