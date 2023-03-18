#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

class TokenBucket {
    protected:
        double b;
        double r;
        double currentBucketLevel;
        int currentPriorityClass;
        high_resolution_clock::time_point lastBucketFillTime;
        int packetCount;
        

    public:
        TokenBucket(double b, double r, int initialPriorityClass);
        int getPriority();
        double getBucketLevel();
        void reportPacketReadyToSend(int payloadSizeBytes);


    protected:
        virtual double getCostOfCurrentPriority();
        void fillBucket();
        int64_t getMicrosSinceLastBucketFill();
        void resetTimeSinceLastBucketRefill();
        virtual void updatePriority();
};