#include <thread>
#include "PendulumSender.h"
#include "../Scheduling/ConstantPriority.h"
#include "../Scheduling/MultiPriorityTokenBucket.h"
#include "SenderConfig.h"
#include "../SerialPortScan/TeensyPortDetector.h"
#include "SenderMultiConfig.h"
#include "MPTBSubConfig.h"
#include "../Parameters/Parameters.h"

/**
 * Usage from command line:
 *
 * Option 1: Use a JSON config file. An example config file can be found in "exampleSenderConfig.json"
 * ./pendulum_sender f <filename>
 *
 * Option 2: Run a sequence of MPTB sub-configs from a JSON config file. An example config file can be found in
 * "exampleSenderSequenceConfig.json".
 * ./pendulum_sender s <filename>
 */

std::string device = "/dev/ttyACM0";

std::string host;
int port = 3000;

// Teensy angle bias of the raw sensor value (2400 steps per revolution)
// this value gets added to the raw sensor value before it is sent to the receiver
int teensyAngleBias;

std::vector<int> networkDelaysPerPrio;

std::string teensyInitializationString;

PendulumSender *sender;


PriorityDeterminer *generateDeterminerFromCommandLineArguments(int argc, char *const *argv);
void runMptbSequence(int argc, char *const *argv);

void sigIntHandler(int signal) {
    std::cout << "Received Signal: " << signal << std::endl;
    sender->stop();
    delete sender;
    exit(0);
}

double numberOfSamplesToBytes(double numberOfSamples) {
    return numberOfSamples * FRAME_SIZE_OF_SAMPLE;
}

double samplingPeriodToDataRate(double samplingPeriod) {
    return (1000.0 / samplingPeriod) * FRAME_SIZE_OF_SAMPLE;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sigIntHandler);

    if(argc >= 2 && argv[1][0] == 's'){
        runMptbSequence(argc, argv);
    } else {
        PriorityDeterminer *determiner;
        determiner = generateDeterminerFromCommandLineArguments(argc, argv);

        sender = new PendulumSender(determiner, device, host, port, teensyInitializationString,
                                    nullptr, "pendulumsender", teensyAngleBias, networkDelaysPerPrio);
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
                                              subConfig.getCosts(), subConfig.getPrioMapping());
}

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
    teensyAngleBias = config.getBias();
    teensyInitializationString = config.getTeensyInitializationString();

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
                sender->sendEndSignal(config.getMptbSubConfigs().at(currentConfigurationIndex-1).getName());
                sender->stop();
                std::cout << "Finished all MPTB sub-configs" << std::endl;
                exit(0);
            }
            double nextConfigRuntimeMinutes = config.getMptbSubConfigs().at(currentConfigurationIndex).getDurationMinutes();
            std::cout << "Starting MPTB sub-config with duration " << nextConfigRuntimeMinutes << " minutes" << std::endl;
            PriorityDeterminer *priorityDeterminer = getIthSubconfigMptbDeterminer(currentConfigurationIndex, config);
            sender->swapPriorityDeterminer(priorityDeterminer,
                                           "pendulumsender_" + config.getMptbSubConfigs().at(currentConfigurationIndex).getName());
            sender->sendNewMptbConfigSignal(currentConfigurationIndex + 1, config.getMptbSubConfigs().at(currentConfigurationIndex-1).getName());
            currentConfigurationStartTime = timeSinceEpochMillisec();
        }
    };

    PriorityDeterminer *determiner = getIthSubconfigMptbDeterminer(0, config);
    sender = new PendulumSender(determiner, device, host, port,
                                teensyInitializationString, regularCallback,
                                "pendulumsender_" + config.getMptbSubConfigs().at(0).getName(),
                                teensyAngleBias, config.getNetworkDelaysPerPrio());
    sender->start();
}

PriorityDeterminer *generateDeterminerFromCommandLineArguments(int argc, char *const *argv) {
    PriorityDeterminer *determiner;

    if (argc >= 3 && argv[1][0] == 'f'){
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
        teensyAngleBias = config.getBias();
        teensyInitializationString = config.getTeensyInitializationString();
        networkDelaysPerPrio = config.getNetworkDelaysPerPrio();

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
        std::cout << "No config file specified. Can't start." << std::endl;
        exit(1);
    }
    return determiner;
}
