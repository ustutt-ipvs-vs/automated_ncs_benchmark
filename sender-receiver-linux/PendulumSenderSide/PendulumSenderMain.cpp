//
// Created by david on 10.01.23.
//

#include <thread>
#include "PendulumSender.h"
#include "../Scheduling/ConstantPriority.h"
#include "../Scheduling/MultiPriorityTokenBucket.h"
#include "SenderConfig.h"
#include "../SerialPortScan/TeensyPortDetector.h"
#include "SenderMultiConfig.h"
#include "MPTBSubConfig.h"

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
 * Option 4: DPTB with thresholds (in samples) and prio mapping
 * ./pendulum_sender t <b (samples)> <r (sampling period)> <amountOfThresholds> <threshold 0> ... <threshold t> p <prioMapping 0> ... <prioMapping t+1> c <costPrio 0> ... <costPrio t+1>
 *
 * Option 5: Use a JSON config file. An example config file can be found in "exampleSenderConfig.json"
 * ./pendulum_sender f <filename>
 *
 * Option 6: Run a sequence of MPTB sub-configs from a JSON config file. An example config file can be found in
 * "exampleSenderSequenceConfig.json".
 * ./pendulum_sender s <filename>
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

// Number of samples that Teensy keeps in its history buffer to determine sampling period.
// Gets transmitted to the sender Teensy at initialization.
int teensyHistorySize = 100;

// Different sampling periods used by the Teensy sender in milliseconds:
std::vector<int> teensySamplingPeriods = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10};

std::vector<double> strictPrioritySamplingPeriods = {90, 80, 70, 60, 50, 40, 30, 20};
std::vector<double> mediumPrioritySamplingPeriods = {75, 59, 43, 26, 10, 9, 8, 7};
std::vector<double> generousPrioritySamplingPeriods = {50, 30, 10, 9, 8, 7, 6, 5};

double strictBucketB = 75; // Samples
double mediumBucketB = 300;
double generousBucketB = 500;

PendulumSender *sender;


PriorityDeterminer *generateDeterminerFromCommandLineArguments(int argc, char *const *argv);
void runMptbSequence(int argc, char *const *argv);

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

    if(argc >= 2 && argv[1][0] == 's'){
        runMptbSequence(argc, argv);
    } else {
        PriorityDeterminer *determiner;
        determiner = generateDeterminerFromCommandLineArguments(argc, argv);

        sender = new PendulumSender(determiner, device, host, port, teensyHistorySize, teensySamplingPeriods);
        sender->start();
    }

}

PriorityDeterminer *getIthSubconfigMptbDeterminer(int i, SenderMultiConfig config){
    MPTBSubConfig subConfig = config.getMptbSubConfigs().at(i);
    double bAsBytes = numberOfSamplesToBytes(subConfig.getB());
    double rAsBytesPerSecond = samplingPeriodToDataRate(subConfig.getR());
    std::vector<double> thresholdsBytes;
    for (double threshold : subConfig.getThresholds()) {
        thresholdsBytes.push_back(numberOfSamplesToBytes(threshold));
    }

    return new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, subConfig.getNumThresholds(),
                                              thresholdsBytes,
                                              subConfig.getCosts(), subConfig.getPrioMapping());}

uint64_t timeSinceEpochMillisec(){
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void runMptbSequence(int argc, char *const *argv){
    if(argc <3){
        std::cout << "Usage: ./pendulum_sender s <multi_mptb_sequence_config_file>" << std::endl;
        exit(1);
    }

    std::string filename = argv[2];
    std::cout << "Using multi-MPTB config file " << filename << std::endl;
    SenderMultiConfig config(filename);

    host = config.getReceiverAddress();
    teensyHistorySize = config.getHistorySize();
    teensySamplingPeriods = config.getSamplingPeriods();

    if(config.isAutomaticallyFindSerialDevice()){
        std::cout << "Automatically finding serial device..." << std::endl;
        device = TeensyPortDetector::findTeensySerialDevice();
    } else {
        device = config.getSerialDeviceName();
    }
    std::cout << "Using serial device " << device << std::endl;

    std::cout << std::endl;
    std::cout << config.toString() << std::endl;

    int currentConfigurationIndex = 0;
    uint64_t currentConfigurationStartTime = timeSinceEpochMillisec();

    auto regularCallback = [&currentConfigurationStartTime, &currentConfigurationIndex, &config]() {
        double currentConfigRuntimeMinutes = config.getMptbSubConfigs().at(currentConfigurationIndex).getDurationMinutes();
        if(timeSinceEpochMillisec() - currentConfigurationStartTime > currentConfigRuntimeMinutes * 60.0 * 1000.0){
            currentConfigurationIndex++;
            if(currentConfigurationIndex >= config.getMptbSubConfigs().size()){
                sender->sendEndSignal();
                sender->stop();
                std::cout << "Finished all MPTB sub-configs" << std::endl;
                exit(0);
            }
            double nextConfigRuntimeMinutes = config.getMptbSubConfigs().at(currentConfigurationIndex).getDurationMinutes();
            std::cout << "Starting MPTB sub-config with duration " << nextConfigRuntimeMinutes << " minutes" << std::endl;
            PriorityDeterminer *priorityDeterminer = getIthSubconfigMptbDeterminer(currentConfigurationIndex, config);
            sender->swapPriorityDeterminer(priorityDeterminer,
                                           "pendulumsender_config_" + std::to_string(currentConfigurationIndex + 1));
            sender->sendNewMptbConfigSignal(currentConfigurationIndex + 1);
            currentConfigurationStartTime = timeSinceEpochMillisec();
        }
    };

    PriorityDeterminer *determiner = getIthSubconfigMptbDeterminer(0, config);
    sender = new PendulumSender(determiner, device, host, port,
                                teensyHistorySize, teensySamplingPeriods, regularCallback, "pendulumsender_config_1");
    sender->start();
}

PriorityDeterminer *generateDeterminerFromCommandLineArguments(int argc, char *const *argv) {
    PriorityDeterminer *determiner;

    // If first argument is 'c' then use constant priority. If it is 'p', use multi priority token bucket:
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
        determiner = new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, 8, dataRates);

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
                r = strictPrioritySamplingPeriods.at(0);  // data rate of the highest priority must be equal to r
            } else if (argv[2][1] == 'm') {
                samplingPeriodsForPriorities = mediumPrioritySamplingPeriods;
                r = mediumPrioritySamplingPeriods.at(0);
            } else if (argv[2][1] == 'g') {
                samplingPeriodsForPriorities = generousPrioritySamplingPeriods;
                r = generousPrioritySamplingPeriods.at(0);
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
        determiner = new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, 8, dataRates);

        std::cout << "Using multi priority token bucket with custom values:" << std::endl;
        std::cout << "b (Samples): " << b << " r (Sampling Period): " << r << std::endl;
        std::cout << "sampling rates for priorities (ms): "
                  << samplingPeriodsForPriorities.at(0) << ", "
                  << samplingPeriodsForPriorities.at(1) << ", "
                  << samplingPeriodsForPriorities.at(2) << ", "
                  << samplingPeriodsForPriorities.at(3) << ", "
                  << samplingPeriodsForPriorities.at(4) << ", "
                  << samplingPeriodsForPriorities.at(5) << ", "
                  << samplingPeriodsForPriorities.at(6) << ", "
                  << samplingPeriodsForPriorities.at(7) << std::endl;
    }
    else if (argc >= 5 && argv[1][0] == 't') {
        double b = std::stod(argv[2]);
        double r = std::stod(argv[3]);
        int numThresholds = std::stoi(argv[4]);
        if (numThresholds < 0 || numThresholds > 8) {
            std::cout << "NumThresholds invalid! At least 1 threshold required and at most 8 thresholds are allowed." << std::endl;
            exit(1);
        }

        std::vector<double> thresholds;
        for (int i = 0; i < numThresholds; i++) {
            thresholds.push_back(numberOfSamplesToBytes(std::stod(argv[i + 5])));
        }
        if (argv[5 + numThresholds][0] != 'p') {
            std::cout << "Invalid argument after last threshold: " << argv[5 + numThresholds] << " - expected 'p'." << std::endl;
            exit(1);
        }

        std::vector<int> prioMapping;
        for (int i = 0; i < numThresholds+1; i++) {
            prioMapping.push_back(std::stoi(argv[i + 6 + numThresholds]));
        }
        if (argv[6 + 2*numThresholds + 1][0] != 'c') {
            std::cout << "Invalid argument after last prioMapping: " << argv[6 + 2*numThresholds + 1] << " - expected 'c'." << std::endl;
            exit(1);
        }

        std::vector<double> costs;
        for (int i = 0; i < numThresholds+1; i++) {
            costs.push_back(std::stod(argv[i + 8 + 2*numThresholds]));         
        }

        std::cout << "Option 4, read these values: b=" << b << ", r=" << r << ", numThresholds=" << numThresholds << ", thresholds=";
        for (int i = 0; i < thresholds.size(); i++) {
            std::cout << thresholds.at(i) << ",";
        }
        std::cout << " prioMappings=";
        for (int i = 0; i < prioMapping.size(); i++) {
            std::cout << prioMapping.at(i) << ",";
        }
        std::cout << " costs=";
        for (int i = 0; i < costs.size(); i++) {
            std::cout << costs.at(i) << ",";
        }
        std::cout << std::endl;

        double bAsBytes = numberOfSamplesToBytes(b);
        double rAsBytesPerSecond = samplingPeriodToDataRate(r);
        
        determiner = new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, numThresholds, thresholds, costs, prioMapping);

    } else if (argc >= 3 && argv[1][0] == 'f'){
        // use config file
        std::string filename = argv[2];
        std::cout << "Using config file " << filename << std::endl;
        SenderConfig config(filename);
        double bAsBytes = numberOfSamplesToBytes(config.getB());
        double rAsBytesPerSecond = samplingPeriodToDataRate(config.getR());
        std::vector<double> thresholdsBytes;
        for (double threshold : config.getThresholds()) {
            thresholdsBytes.push_back(numberOfSamplesToBytes(threshold));
        }

        determiner = new MultiPriorityTokenBucket(bAsBytes, rAsBytesPerSecond, config.getNumThresholds(),
                                                  thresholdsBytes,
                                                  config.getCosts(), config.getPrioMapping());

        host = config.getReceiverAddress();
        teensyHistorySize = config.getHistorySize();
        teensySamplingPeriods = config.getSamplingPeriods();

        if(config.isAutomaticallyFindSerialDevice()){
            std::cout << "Automatically finding serial device..." << std::endl;
            device = TeensyPortDetector::findTeensySerialDevice();
        } else {
            device = config.getSerialDeviceName();
        }
        std::cout << "Using serial device " << device << std::endl;

        std::cout << config.toString() << std::endl;
    }

    else {
        std::vector<double> dataRates = samplingPeriodsToDataRates(
                {100, 84, 67, 50, 40, 30, 20, 10});
        determiner = new MultiPriorityTokenBucket(7100, 710, 8, dataRates);

        std::cout << "Warning: Using default parameters" << std::endl;
    }
    return determiner;
}
