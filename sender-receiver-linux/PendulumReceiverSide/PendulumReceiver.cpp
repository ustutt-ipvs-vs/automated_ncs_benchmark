//
// Created by david on 11.01.23.
//

#include "PendulumReceiver.h"

PendulumReceiver::PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort, bool doPauses)
        : logger("pendulumreceiver") {
    this->serialDeviceName = serialDeviceName;
    this->receiverAddress = inet_address(receiverHost, receiverPort);
    this->doPauses = doPauses;

    receiverSocket.bind(receiverAddress);

    serialActuator = SerialPort(serialDeviceName, BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE,
                                NumStopBits::ONE);
    serialActuator.SetTimeout(-1);
}

void PendulumReceiver::start() {
    serialActuator.Open();

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

