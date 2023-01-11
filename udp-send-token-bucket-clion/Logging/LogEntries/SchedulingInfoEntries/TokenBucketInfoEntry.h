//
// Created by david on 11.01.23.
//

#ifndef UDP_SEND_TOKEN_BUCKET_CLION_TOKENBUCKETINFOENTRY_H
#define UDP_SEND_TOKEN_BUCKET_CLION_TOKENBUCKETINFOENTRY_H


#include "SchedulingInfoEntry.h"
#include "../../../TokenBucket.hpp"

class TokenBucketInfoEntry : public SchedulingInfoEntry{
private:
    double b, r;
    int priority;
    double bucketLevel;

public:
    TokenBucketInfoEntry(TokenBucket* tokenBucket);

    double getB() const;

    double getR() const;

    int getPriority() const;

    double getBucketLevel() const;

    nlohmann::json toJson() override;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_TOKENBUCKETINFOENTRY_H
