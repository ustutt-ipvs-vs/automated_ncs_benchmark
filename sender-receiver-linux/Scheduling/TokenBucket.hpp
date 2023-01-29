#ifndef TOKEN_BUCKET_H
#define TOKEN_BUCKET_H

#include <chrono>
#include "PriorityDeterminer.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

class TokenBucket : public PriorityDeterminer {
    protected:
        double b;   // In bytes
        double r;   // in bytes per second
        double currentBucketLevel;
        int currentPriorityClass;
        high_resolution_clock::time_point lastBucketFillTime;
        int packetCount;

public:
    double getB() const;

    double getR() const;

public:
        TokenBucket(double b, double r, int initialPriorityClass);
        int getPriority() override;
        double getBucketLevel();
        void reportPacketReadyToSend(int payloadSizeBytes) override;
        SchedulingInfoEntry * getSchedulingInfoEntry() override;
        std::string getDebugInfoString() override;
        void resetState() override;


    protected:
        virtual double getCostOfCurrentPriority() = 0;
        void fillBucket();
        int64_t getMicrosSinceLastBucketFill();
        void resetTimeSinceLastBucketRefill();
        virtual void updatePriority() = 0;
};

#endif