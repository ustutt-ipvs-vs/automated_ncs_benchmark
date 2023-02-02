//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"
#include "../Scheduling/ConstantPriority.h"
#include "../Scheduling/MultiPriorityTokenBucket.h"

std::string device = "/dev/ttyACM0";

std::string host = "10.0.1.2";
int port = 3000;

double frameSizeOfSample = 68.6; // 22.6B payload + 14B Ethernet header + 4B VLAN tag + 20B IP header + 8B UDP header

PendulumSender *sender;


void sigIntHandler(int signal) {
    std::cout << "Received Signal: " << signal << std::endl;
    sender->stop();
    delete sender;
    exit(0);
}

std::vector<double> samplingPeriodsToDataRates(std::vector<double> samplingPeriods) {
    std::vector<double> dataRates;
    for (double samplingPeriod: samplingPeriods) {
        double dataRate = (1000.0 / samplingPeriod) * frameSizeOfSample;
        dataRates.emplace_back(dataRate);
    }
    return dataRates;
}

double numberOfSamplesToBytes(double numberOfSamples) {
    return numberOfSamples * frameSizeOfSample;
}

double samplingPeriodToDataRate(double samplingPeriod) {
    return (1000.0 / samplingPeriod) * frameSizeOfSample;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sigIntHandler);

    PriorityDeterminer *determiner;

    // If first argument ist 'c' then use constant priority. If it is 'p', use multi priority token bucket:
    if (argc >= 3 && argv[1][0] == 'c') {
        int priority = std::stoi(argv[2]);
        determiner = new ConstantPriority(priority);

        std::cout << "Using constant priority " << priority << std::endl;
    } else if (argc >= 12 && argv[1][0] == 'p') {
        double b = std::stod(argv[2]);
        double r = std::stod(argv[3]);
        double prio0SamplingPeriod = std::stod(argv[4]);
        double prio1SamplingPeriod = std::stod(argv[5]);
        double prio2SamplingPeriod = std::stod(argv[6]);
        double prio3SamplingPeriod = std::stod(argv[7]);
        double prio4SamplingPeriod = std::stod(argv[8]);
        double prio5SamplingPeriod = std::stod(argv[9]);
        double prio6SamplingPeriod = std::stod(argv[10]);
        double prio7SamplingPeriod = std::stod(argv[11]);

        double bAsBytes = numberOfSamplesToBytes(b);
        double rAsBytesPerSecond = samplingPeriodToDataRate(r);
        std::vector<double> dataRates = samplingPeriodsToDataRates(
                {prio0SamplingPeriod, prio1SamplingPeriod, prio2SamplingPeriod, prio3SamplingPeriod,
                 prio4SamplingPeriod,
                 prio5SamplingPeriod, prio6SamplingPeriod, prio7SamplingPeriod});
        determiner = new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, 8, 0, dataRates);

        std::cout << "Using multi priority token bucket with custom values:" << std::endl;
        std::cout << "b (Samples): " << b << " r (Sampling Period): " << r << std::endl;
        std::cout << "sampling rates for priorities (ms): " << prio0SamplingPeriod << ", " << prio1SamplingPeriod << ", "
                  << prio2SamplingPeriod << ", " << prio3SamplingPeriod << ", " << prio4SamplingPeriod << ", "
                  << prio5SamplingPeriod << ", " << prio6SamplingPeriod << ", " << prio7SamplingPeriod << std::endl;
    } else {
        determiner = new MultiPriorityTokenBucket(7100, 710, 8, 0, {100, 84, 67, 50, 40, 30, 20, 10});

        std::cout << "Warning: Using default parameters" << std::endl;
    }

    sender = new PendulumSender(determiner, device, host, port);
    sender->start();
}
