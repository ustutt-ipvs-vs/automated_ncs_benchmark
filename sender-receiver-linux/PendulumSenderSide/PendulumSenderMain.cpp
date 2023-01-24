//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"

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

int main(){
    signal(SIGINT, sigIntHandler);
    PriorityDeterminer* determiner = new TokenBucketPrioTest(b, r, initialPriority);
    sender = new PendulumSender(determiner, device, host, port);
    sender->start();
}