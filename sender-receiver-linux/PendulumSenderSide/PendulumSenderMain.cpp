//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"
#include "../Scheduling/ConstantPriority.h"
#include "../Scheduling/MultiPriorityTokenBucket.h"

/**
 * Usage from command line:
 *
 * Option 1: Constant Priority
 * ./pendulum_sender c <priority>
 *
 * Option 2: Multi Priority Token Bucket with custom parameters
 * ./pendulum_sender p <b (samples)> <r (sampling period)> <priority 0 sampling period (ms)> ... <priority 7 sampling period (ms)>
 *
 * Option 3: Multi Priority Token Bucket with selection of parameters:
 * ./pendulum_sender m <priority mapping mode> <token bucket mode>
 *
 * where priority mapping mode is one of
 * ps: strict priority sampling periods
 * pm: medium priority sampling periods
 * pg: generous priority sampling periods
 *
 * and token bucket mode is one of
 * bs: strict bucket
 * bm: medium bucket
 * bg: generous bucket
 *
 * Running the program as sudo (required for priority 7 to work):
 * $ sudo su
 * $ export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
 * $ ./pendulum_sender [arguments]
 */

std::string device = "/dev/ttyACM0";

std::string host = "10.0.1.2";
int port = 3000;

// 78B = 32B payload + 14B Ethernet header + 4B VLAN tag + 20B IP header + 8B UDP header
double frameSizeOfSample = 78;

std::vector<double> strictPrioritySamplingPeriods = {90, 80, 70, 60, 50, 40, 30, 20};
std::vector<double> mediumPrioritySamplingPeriods = {75, 59, 43, 26, 10, 9, 8, 7};
std::vector<double> generousPrioritySamplingPeriods = {50, 30, 10, 9, 8, 7, 6, 5};

double strictBucketB = 75; // Samples
double mediumBucketB = 300;
double generousBucketB = 500;

PendulumSender *sender;


PriorityDeterminer *generateDeterminerFromCommandLineArguments(int argc, char *const *argv);

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
    determiner = generateDeterminerFromCommandLineArguments(argc, argv);

    sender = new PendulumSender(determiner, device, host, port);
    sender->start();
}

PriorityDeterminer *generateDeterminerFromCommandLineArguments(int argc, char *const *argv) {
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
        std::cout << "sampling rates for priorities (ms): " << prio0SamplingPeriod << ", " << prio1SamplingPeriod
                  << ", "
                  << prio2SamplingPeriod << ", " << prio3SamplingPeriod << ", " << prio4SamplingPeriod << ", "
                  << prio5SamplingPeriod << ", " << prio6SamplingPeriod << ", " << prio7SamplingPeriod << std::endl;
    } else if (argc >= 4 && argv[1][0] == 'm') {
        double b, r;
        std::vector<double> samplingPeriodsForPriorities;

        // Second argument determines which sampling periods to use.
        // If it is 'ps', use strict priority. If it is 'pm', use medium priority. If it is 'pg', use generous priority.
        // Only 'pm', 'pg' and 'ps' are supported.
        if (argv[2][0] == 'p') {
            if (argv[2][1] == 's') {
                samplingPeriodsForPriorities = strictPrioritySamplingPeriods;
                r = strictPrioritySamplingPeriods[0];  // data rate of the highest priority must be equal to r
            } else if (argv[2][1] == 'm') {
                samplingPeriodsForPriorities = mediumPrioritySamplingPeriods;
                r = mediumPrioritySamplingPeriods[0];
            } else if (argv[2][1] == 'g') {
                samplingPeriodsForPriorities = generousPrioritySamplingPeriods;
                r = generousPrioritySamplingPeriods[0];
            } else {
                std::cout << "Invalid argument: " << argv[2] << std::endl;
                exit(1);
            }
        } else {
            std::cout << "Invalid argument: " << argv[2] << std::endl;
            exit(1);
        }

        // Third argument determines the bucket size. If it is 'bs', use small bucket. If it is 'bm', use medium bucket.
        // If it is 'bg', use generous bucket.
        // Only 'bs', 'bm' and 'bg' are supported.
        if (argv[3][0] == 'b') {
            if (argv[3][1] == 's') {
                b = strictBucketB;
            } else if (argv[3][1] == 'm') {
                b = mediumBucketB;
            } else if (argv[3][1] == 'g') {
                b = generousBucketB;
            } else {
                std::cout << "Invalid argument: " << argv[3] << std::endl;
                exit(1);
            }
        } else {
            std::cout << "Invalid argument: " << argv[3] << std::endl;
            exit(1);
        }

        std::vector<double> dataRates = samplingPeriodsToDataRates(samplingPeriodsForPriorities);
        double bAsBytes = numberOfSamplesToBytes(b);
        double rAsBytesPerSecond = samplingPeriodToDataRate(r);
        determiner = new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, 8, 0, dataRates);

        std::cout << "Using multi priority token bucket with custom values:" << std::endl;
        std::cout << "b (Samples): " << b << " r (Sampling Period): " << r << std::endl;
        std::cout << "sampling rates for priorities (ms): "
                  << samplingPeriodsForPriorities[0] << ", "
                  << samplingPeriodsForPriorities[1] << ", "
                  << samplingPeriodsForPriorities[2] << ", "
                  << samplingPeriodsForPriorities[3] << ", "
                  << samplingPeriodsForPriorities[4] << ", "
                  << samplingPeriodsForPriorities[5] << ", "
                  << samplingPeriodsForPriorities[6] << ", "
                  << samplingPeriodsForPriorities[7] << std::endl;
    } else {
        std::vector<double> dataRates = samplingPeriodsToDataRates(
                {100, 84, 67, 50, 40, 30, 20, 10});
        determiner = new MultiPriorityTokenBucket(7100, 710, 8, 0, dataRates);

        std::cout << "Warning: Using default parameters" << std::endl;
    }
    return determiner;
}
