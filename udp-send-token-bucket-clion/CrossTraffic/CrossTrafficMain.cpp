#include "CrossTrafficSenderOrchestrator.h"

CrossTrafficSenderOrchestrator orchestrator;

void sigIntHandler(int signal){
    std::cout << "Received Signal: " << signal << std::endl;
    orchestrator.stop();
    exit(0);
}

int main(){
    orchestrator.initializeSendersByDefault();
    orchestrator.start();
    std::cout << "Started now" << std::endl;

    signal(SIGINT, sigIntHandler);

    while(true){} // Keep main thread alive while other threads run
}
