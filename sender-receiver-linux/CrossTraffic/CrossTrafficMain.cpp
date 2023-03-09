#include "CrossTrafficSenderOrchestrator.h"

CrossTrafficSenderOrchestrator orchestrator;

void sigIntHandler(int signal){
    std::cout << "Received Signal: " << signal << std::endl;
    orchestrator.stop();
    exit(0);
}

int main(int argc, char *argv[]){
    std::string host = "10.0.1.3";
    int port = 3000;
    std::vector<int> priorities;
    std::vector<int> dataRatesBytesPerSecond;

    // If the first argument is prio, then use the following arguments as priorities and data rates:
    // Priorities: 0, 1, 2, 3, 4, 5, 6, 7
    // Data rates: 2 Mbit/s for all 8 senders
    //
    // Else if the first argument is fifo, then use the following arguments as data rates:
    // Priorities: 0 for all 8 senders
    // Data rate: 2 Mbit/s for all 8 senders
    //
    // Else if the first argument is custom, then the next arguments have to be in the following format:
    // Priority1:DataRate1 Priority2:DataRate2 Priority3:DataRate3
    if (argc > 1 && std::string(argv[1]) == "prio") {
        for (int i = 0; i < 8; i++) {
            priorities.emplace_back(i);
            dataRatesBytesPerSecond.emplace_back(250'000);
        }
    } else if (argc > 1 && std::string(argv[1]) == "fifo") {
        for (int i = 0; i < 8; i++) {
            priorities.emplace_back(0);
            dataRatesBytesPerSecond.emplace_back(250'000);
        }
    } else if (argc > 1 && std::string(argv[1]) == "custom") {
        for (int i = 2; i < argc; i++) {
            std::string arg = argv[i];
            std::string delimiter = ":";
            std::string priority = arg.substr(0, arg.find(delimiter));
            std::string dataRate = arg.substr(arg.find(delimiter) + 1, arg.length());
            priorities.emplace_back(std::stoi(priority));
            dataRatesBytesPerSecond.emplace_back(std::stoi(dataRate));
        }
    } else {
        std::cout << "Usage: " << argv[0] << " prio|fifo|custom [Priority1:DataRate1 Priority2:DataRate2 Priority3:DataRate3 ...]" << std::endl;
        exit(1);
    }

    // Print the priorities and data rates:
    std::cout << "Priorities: ";
    for (int priority: priorities) {
        std::cout << priority << " ";
    }
    std::cout << std::endl;
    std::cout << "Data rates: ";
    for (int dataRate: dataRatesBytesPerSecond) {
        std::cout << dataRate << " ";
    }
    std::cout << std::endl;

    orchestrator.initializeSendersWithParameters(host, port, priorities, dataRatesBytesPerSecond);

    orchestrator.start();
    std::cout << "Started now" << std::endl;

    signal(SIGINT, sigIntHandler);

    while(true){} // Keep main thread alive while other threads run
}

