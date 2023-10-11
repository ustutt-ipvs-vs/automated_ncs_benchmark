//
// Created by david on 29.01.23.
//

#ifndef SENDER_RECEIVER_LINUX_MULTIPRIORITYTOKENBUCKET_H
#define SENDER_RECEIVER_LINUX_MULTIPRIORITYTOKENBUCKET_H


#include "PriorityDeterminer.h"
using std::chrono::high_resolution_clock;

class MultiPriorityTokenBucket : public PriorityDeterminer {
protected:
    double b;   // In bytes
    double r;   // in bytes per second
    double currentBucketLevel;
    unsigned int currentSeverityLevel;
    high_resolution_clock::time_point lastBucketFillTime;
    unsigned int packetCount;

public:
    MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities,
                             std::vector<double> dataRateOfPriorities);

    MultiPriorityTokenBucket(double b, double r, unsigned int numPriorities,
                             std::vector<double> thresholds, std::vector<double> costs, std::vector<int> prioMapping);

    unsigned int getPriority() override;

    double getBucketLevel();
    void reportPacketReadyToSend(int payloadSizeBytes) override;
    SchedulingInfoEntry* getSchedulingInfoEntry() override;
    std::string getDebugInfoString() override;
    void resetState() override;

    double getB() const;
    double getR() const;


protected:

    void fillBucket();
    int64_t getMicrosSinceLastBucketFill();

    void updateSeverityLevel();
    double getCostOfCurrentPriority();

private:
    void calculateThresholdsAndCosts(double r, std::vector<double> dataRateOfPriorities);

    std::vector<double> thresholdOfPriorities;
    std::vector<double> costOfPriorities;
    std::vector<int> prioMapping;
    unsigned int numPriorities;
};


#endif //SENDER_RECEIVER_LINUX_MULTIPRIORITYTOKENBUCKET_H
