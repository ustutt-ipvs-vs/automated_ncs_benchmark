#ifndef SENDER_RECEIVER_LINUX_MPTBSUBCONFIG_H
#define SENDER_RECEIVER_LINUX_MPTBSUBCONFIG_H

#include <vector>
#include "../nlohmann/json.hpp"

class MPTBSubConfig {
private:
    double durationMinutes;
    double b;
    double r;
    int numThresholds;
    std::vector<double> thresholds;
    std::vector<int> prioMapping;
    std::vector<double> costs;

public:
    MPTBSubConfig(nlohmann::json configJson);

    double getDurationMinutes() const;

    double getB() const;

    double getR() const;

    int getNumThresholds() const;

    const std::vector<double> &getThresholds() const;

    const std::vector<int> &getPrioMapping() const;

    const std::vector<double> &getCosts() const;

    std::string toString() const;

};


#endif //SENDER_RECEIVER_LINUX_MPTBSUBCONFIG_H
