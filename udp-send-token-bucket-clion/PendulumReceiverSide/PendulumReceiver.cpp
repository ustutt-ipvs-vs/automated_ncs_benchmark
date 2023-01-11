//
// Created by david on 11.01.23.
//

#include "PendulumReceiver.h"

PendulumReceiver::PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort)
        : logger("pendulumreceiver") {
    this->serialDeviceName = serialDeviceName;
    this->receiverAddress = inet_address(receiverHost, receiverPort);

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
        logger.log(packetCount, bytesReceivedTotal, networkInput);

        serialActuator.Write(networkInput);

        while (serialActuator.Available() > 0) {
            serialActuator.Read(serialInput);
            std::cout << "Actuator: " << serialInput << std::endl;
        }
    }
}

void PendulumReceiver::stop() {
    stopReceiving = true;
    receiverSocket.close();
    serialActuator.Close();
}
