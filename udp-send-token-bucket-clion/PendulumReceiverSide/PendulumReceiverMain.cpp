//
// Created by david on 11.01.23.
//

#include <string>
#include "PendulumReceiver.h"

std::string device = "/dev/ttyACM0";

std::string host = "10.0.1.2";
int port = 3000;

PendulumReceiver receiver(device, host, port);

void sigIntHandler(int signal){
    std::cout << "Received Signal: " << signal << std::endl;
    receiver.stop();
    exit(0);
}

int main(){
    signal(SIGINT, sigIntHandler);
    receiver.start();
}
