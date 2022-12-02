#ifndef TOKEN_BUCKET_PRIO_TEST_H
#define TOKEN_BUCKET_PRIO_TEST_H

#include "TokenBucket.hpp"

class TokenBucketPrioTest: public TokenBucket {
    protected:
    double getCostOfCurrentPriority() override;
        void updatePriority() override;

public:
    TokenBucketPrioTest(double b, double r, int initialPriorityClass);
};

#endif
