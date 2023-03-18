#include "TokenBucket.hpp"

class TokenBucketConstantPrio: TokenBucket {
    protected:
        virtual double getCostOfCurrentPriority();
        virtual void updatePriority();
};