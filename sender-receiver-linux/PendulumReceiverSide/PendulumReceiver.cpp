//
// Created by david on 11.01.23.
//

#include "PendulumReceiver.h"

PendulumReceiver::PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort,
                                   bool doPauses, int timeBetweenPausesMillis, int pauseDurationMillis,
                                   int motorMaxRPM, double revolutionsPerTrack)
        : logger("pendulumreceiver") {
    this->serialDeviceName = serialDeviceName;
    this->receiverAddress = inet_address(receiverHost, receiverPort);
    this->doPauses = doPauses;
    this->timeBetweenPausesMillis = timeBetweenPausesMillis;
    this->pauseDurationMillis = pauseDurationMillis;
    this->motorMaxRPM = motorMaxRPM;
    this->revolutionsPerTrack = revolutionsPerTrack;

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
    // Create string of form "I:motorMaxRPM;revolutionsPerTrack;\n"
    std::string teensyInitParams = "I:" + std::to_string(motorMaxRPM) + ";" + std::to_string(revolutionsPerTrack) + ";\n";
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

        packetCount++;
        bytesReceivedTotal += networkInput.size();

        // Remove the padding with '#' from the end of the string:
        networkInput.erase(std::remove(networkInput.begin(), networkInput.end(), '#'), networkInput.end());

        logger.log(packetCount, bytesReceivedTotal, networkInput);

        if(doPauses && isTimeForPause()){
            sendPauseSignal();
        }
        serialActuator.Write(networkInput);

        while (serialActuator.Available() > 0) {
            serialInput.clear();
            serialActuator.Read(serialInput);
            std::cout << "Actuator: " << serialInput << std::endl;

            if (serialInput.rfind("log:", 0) == 0) {
                logger.logActuator(serialInput);

                if(!startedBalancing) {
                    startedBalancing = true;
                    lastPauseTime = high_resolution_clock::now();
                }
            }
        }
    }
}

void PendulumReceiver::sendPauseSignal() {
    // The pause signal has the form (for 800ms pause duration):
    // PAUSE:800;
    lastPauseTime = high_resolution_clock::now();
    std::stringstream ss;
    ss << "PAUSE:" << pauseDurationMillis << ";\n";
    serialActuator.Write(ss.str());
    logger.logPause(pauseDurationMillis);
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
    logger.saveToFile();
}



