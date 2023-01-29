//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"
#include "../Scheduling/ConstantPriority.h"
#include "../Scheduling/MultiPriorityTokenBucket.h"

std::string device = "/dev/ttyACM0";

std::string host = "10.0.1.2";
int port = 3000;

double b = 2'000;
double r = 1'000;
int initialPriority = 0;

PendulumSender* sender;


void sigIntHandler(int signal){
    std::cout << "Received Signal: " << signal << std::endl;
    sender->stop();
    delete sender;
    exit(0);
}

std::vector<double> samplingPeriodsToDataRates(std::vector<double> samplingPeriods){
    std::vector<double> dataRates;
    int frameSizeOfSample = 71;
    for(double samplingPeriod : samplingPeriods){
        double dataRate = (1000.0 / samplingPeriod) * frameSizeOfSample;
        dataRates.emplace_back(dataRate);
    }
    return dataRates;
}

int main(){
    signal(SIGINT, sigIntHandler);
    //PriorityDeterminer* determiner = new TokenBucketPrioTest(b, r, initialPriority);
    //PriorityDeterminer* determiner = new ConstantPriority(1);
    //PriorityDeterminer* determiner = new TokenBucketPrioTest(7100, 710, 0);

    std::vector<double> dataRates = samplingPeriodsToDataRates({100, 84, 67, 50, 40, 30, 20, 10});
    PriorityDeterminer* determiner = new MultiPriorityTokenBucket(7100, 710, 8, 0, dataRates);


    sender = new PendulumSender(determiner, device, host, port);
    sender->start();
}