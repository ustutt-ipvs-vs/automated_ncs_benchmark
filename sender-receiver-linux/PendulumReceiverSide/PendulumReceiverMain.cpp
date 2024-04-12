/**
 * Usage:
 * ./pendulum_receiver config_file.json
 */

#include <string>
#include "PendulumReceiver.h"
#include "../SerialPortScan/TeensyPortDetector.h"

std::string device = "/dev/ttyACM0";

std::string host = "10.0.1.2";
int port = 3000;

PendulumReceiver* receiver;

void sigIntHandler(int signal){
    std::cout << "Received Signal: " << signal << std::endl;
    receiver->stop();
    delete receiver;
    exit(0);
}

int main(int argc, char *argv[]){
    signal(SIGINT, sigIntHandler);

    if(argc >= 2) {
        std::string configFile = argv[1];
        ReceiverConfig config(configFile);
        if(config.isAutomaticallyFindSerialDevice()){
            device = TeensyPortDetector::findTeensySerialDevice();
        } else{
            device = config.getSerialDeviceName();
        }

        receiver = new PendulumReceiver(device, config.getReceiverAddress(), port,
                                        config.getMotorMaxRPM(),config.getRevolutionsPerTrack(),
                                        config.getSwingUpDistanceFactor(),config.getSwingUpSpeedFactor(), config.getSwingUpAccelerationFactor(),
                                        config.getSwingUpBehavior(), config.getKalmanAndControllerParameterString());

        std::cout << "Using config file: " << configFile << std::endl;
        std::cout << config.toString() << std::endl;

    } else {
        std::cout << "No config file specified, using default values." << std::endl;
        receiver = new PendulumReceiver(device, host, port, true, 20'000, 800, 20*60, 20.06, 0.09, 1.0, 1.0, ReceiverConfig::SWING_UP_AT_START);
    }
    receiver->start();
}
