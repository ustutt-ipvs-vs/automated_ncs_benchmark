//
// Created by david on 11.01.23.
//

#include "PendulumReceiver.h"

PendulumReceiver::PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort,
                                   bool doPauses, int timeBetweenPausesMillis, int pauseDurationMillis,
                                   int motorMaxRPM, double revolutionsPerTrack,
                                   ReceiverConfig::SwingUpBehavior swingUpBehavior){
    this->serialDeviceName = serialDeviceName;
    this->receiverAddress = inet_address(receiverHost, receiverPort);
    this->doPauses = doPauses;
    this->timeBetweenPausesMillis = timeBetweenPausesMillis;
    this->pauseDurationMillis = pauseDurationMillis;
    this->motorMaxRPM = motorMaxRPM;
    this->revolutionsPerTrack = revolutionsPerTrack;
    this->swingUpBehavior = swingUpBehavior;
    this->logger = new PendulumLogger("pendulumreceiver_config_1");

    receiverSocket.bind(receiverAddress);

    serialActuator = SerialPort(serialDeviceName, BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE,
                                NumStopBits::ONE);
    serialActuator.SetTimeout(-1);
}

void PendulumReceiver::start() {
    serialActuator.Open();

    // Wait for actuator to be ready:
    std::cout << "Waiting for actuator Teensy to send READY signal." << std::endl;
    serialActuator.Read(serialInput);
    while (serialInput.rfind("READY", 0) != 0) {
        serialInput.clear();
        serialActuator.Read(serialInput);
    }

    // Send Teensy drive geometry parameters:
    std::cout << "Sending drive geometry parameters: motor max RPM: " << motorMaxRPM << ", revolutions per track: "
        << revolutionsPerTrack << std::endl;
    // Create string of form "I:motorMaxRPM;revolutionsPerTrack;swingUpAtStart\n"

    uint8_t swingUpAtStart = swingUpBehavior == ReceiverConfig::SwingUpBehavior::SWING_UP_AT_START
            || swingUpBehavior == ReceiverConfig::SwingUpBehavior::CRASH_AND_SWING_UP_AT_NEW_CONFIG
            || swingUpBehavior == ReceiverConfig::SwingUpBehavior::SWING_UP_AT_NEW_CONFIG_IF_CRASHED;

    std::string teensyInitParams = "I:"
            + std::to_string(motorMaxRPM) + ";"
            + std::to_string(revolutionsPerTrack) + ";"
            + std::to_string(swingUpAtStart) + ";\n";
    serialActuator.Write(teensyInitParams);

    // Wait for first serial values to arrive, before going into main loop:
    std::cout << "Waiting for actuator to start" << std::endl;
    serialInput.clear();
    serialActuator.Read(serialInput);
    std::cout << "Actuator: " << serialInput << std::endl;

    // Initialization complete. Main loop:
    while (!stopReceiving) {
        int receivedLength = receiverSocket.recv(receiveBuffer, sizeof(receiveBuffer));
        networkInput = std::string(receiveBuffer, receivedLength);

        // Detect new config signal of the form "NewConfig:number\n":
        if (networkInput.rfind("NewConfig:", 0) == 0) {
            handleNewConfigSignal();
            continue;
        }

        // Detect end signal of the form "EndSignal\n":
        if (networkInput.rfind("EndSignal", 0) == 0) {
            std::cout << "End signal received." << std::endl;
            stop();
            continue;
        }

        packetCount++;
        bytesReceivedTotal += networkInput.size();

        // Remove the padding with '#' from the end of the string:
        networkInput.erase(std::remove(networkInput.begin(), networkInput.end(), '#'), networkInput.end());

        logger->log(packetCount, bytesReceivedTotal, networkInput);

        if(doPauses && isTimeForPause()){
            sendPauseSignal();
        }
        serialActuator.Write(networkInput);

        while (serialActuator.Available() > 0) {
            serialInput.clear();
            serialActuator.Read(serialInput);
            std::cout << "Actuator: " << serialInput << std::endl;

            if (serialInput.rfind("log:", 0) == 0) {
                logger->logActuator(serialInput);

                if(!startedBalancing) {
                    startedBalancing = true;
                    lastPauseTime = high_resolution_clock::now();
                }
            } else if(serialInput.rfind("CT:GracePeriodEnded;", 0) == 0){
                std::cout << "Grace period has ended. Resetting logger now." << std::endl;
                logger->reset();
            }
        }
    }
}

void PendulumReceiver::handleNewConfigSignal() {
    int newConfigNumber = std::stoi(networkInput.substr(10));
    std::cout << "New MPTB config signal received: " << newConfigNumber << std::endl;
    startNewLogfile(newConfigNumber);

    if(swingUpBehavior == ReceiverConfig::CRASH_AND_SWING_UP_AT_NEW_CONFIG){
        std::cout << "Sending signal to crash and swing up again." << std::endl;
        serialActuator.Write("DOCRASHANDSWINGUP:1;\n");
    } else if(swingUpBehavior == ReceiverConfig::SWING_UP_AT_NEW_CONFIG_IF_CRASHED){
        std::cout << "Sending signal to swing up if is crashed." << std::endl;
        serialActuator.Write("DOSWINGUP:1;\n");
    }
}

void PendulumReceiver::sendPauseSignal() {
    // The pause signal has the form (for 800ms pause duration):
    // PAUSE:800;
    lastPauseTime = high_resolution_clock::now();
    std::stringstream ss;
    ss << "PAUSE:" << pauseDurationMillis << ";\n";
    serialActuator.Write(ss.str());
    logger->logPause(pauseDurationMillis);
}

bool PendulumReceiver::isTimeForPause() const {
    if(!startedBalancing){
        return false;
    }
    auto currentTime = high_resolution_clock::now();
    return std::chrono::duration_cast<milliseconds>(currentTime - lastPauseTime).count() > timeBetweenPausesMillis;
}

void PendulumReceiver::stop() {
    stopReceiving = true;
    receiverSocket.close();
    serialActuator.Close();
    logger->saveToFile();
}

void PendulumReceiver::startNewLogfile(int number) {
    logger->saveToFileAsync(); // Save asynchronously to avoid blocking the main thread
    logger = new PendulumLogger("pendulumreceiver_config_" + std::to_string(number));
}



