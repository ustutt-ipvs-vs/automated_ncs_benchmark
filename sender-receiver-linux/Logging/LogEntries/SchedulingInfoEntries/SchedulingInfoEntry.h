#ifndef UDP_SEND_TOKEN_BUCKET_CLION_SCHEDULINGINFOENTRY_H
#define UDP_SEND_TOKEN_BUCKET_CLION_SCHEDULINGINFOENTRY_H


#include "../../../nlohmann/json.hpp"

class SchedulingInfoEntry {
public:
    virtual nlohmann::json toJson() = 0;
};


#endif //UDP_SEND_TOKEN_BUCKET_CLION_SCHEDULINGINFOENTRY_H
