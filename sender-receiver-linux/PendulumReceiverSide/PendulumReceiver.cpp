#include "PendulumReceiver.h"

PendulumReceiver::PendulumReceiver(std::string serialDeviceName, std::string receiverHost, int receiverPort,
                                   int motorMaxRPM, double revolutionsPerTrack, float swingUpDistanceFactor,
                                   float swingUpSpeedFactor, float swingUpAccelerationFactor,
                                   ReceiverConfig::SwingUpBehavior swingUpBehavior,
                                   std::string kalmanAndControllerParamString){
    this->serialDeviceName = serialDeviceName;
    this->receiverAddress = inet_address(receiverHost, receiverPort);
    this->motorMaxRPM = motorMaxRPM;
    this->revolutionsPerTrack = revolutionsPerTrack;
    this->swingUpBehavior = swingUpBehavior;
    this->kalmanAndControllerParamString = kalmanAndControllerParamString;
    this->swingUpDistanceFactor = swingUpDistanceFactor;
    this->swingUpSpeedFactor = swingUpSpeedFactor;
    this->swingUpAccelerationFactor = swingUpAccelerationFactor;
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

    uint8_t swingUpAtStart = swingUpBehavior == ReceiverConfig::SwingUpBehavior::SWING_UP_AT_START
            || swingUpBehavior == ReceiverConfig::SwingUpBehavior::CRASH_AND_SWING_UP_AT_NEW_CONFIG
            || swingUpBehavior == ReceiverConfig::SwingUpBehavior::SWING_UP_AT_NEW_CONFIG_IF_CRASHED;

    // Create string of the form
    // "I:motorMaxRPM;revolutionsPerTrack;doSwingUpAtStart;swingUpDistanceFactor;swingUpSpeedFactor;swingUpAccelerationFactor;K_1;K_2;K_3;K_4;K_integrator;approach\n"
    std::string teensyInitParams = "I:"
            + std::to_string(motorMaxRPM) + ";"
            + std::to_string(revolutionsPerTrack) + ";"
            + std::to_string(swingUpAtStart) + ";"
            + std::to_string(swingUpDistanceFactor) + ";"
            + std::to_string(swingUpSpeedFactor) + ";"
            + std::to_string(swingUpAccelerationFactor) + ";"
            + kalmanAndControllerParamString + "\n";
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
            handleEndSignal();
            continue;
        }

        packetCount++;
        bytesReceivedTotal += networkInput.size();

        // Remove the padding with '#' from the end of the string:
        networkInput.erase(std::remove(networkInput.begin(), networkInput.end(), '#'), networkInput.end());

        logger->log(packetCount, bytesReceivedTotal, networkInput);

        serialActuator.Write(networkInput);

        while (serialActuator.Available() > 0) {
            serialInput.clear();
            serialActuator.Read(serialInput);
            std::cout << "Actuator: " << serialInput << std::endl;

            if (serialInput.rfind("log:", 0) == 0) {
                logger->logActuator(serialInput);

                if(!startedBalancing) {
                    startedBalancing = true;
                }
            } else if(serialInput.rfind("CT:GracePeriodEnded;") < serialInput.size()){
                std::cout << "Grace period has ended. Resetting logger now." << std::endl;
                logger->reset();
            }
        }
    }
}

/**
 * Handles the new config signal of the form "NewConfig:number;previousConfigName\n"
 */
void PendulumReceiver::handleNewConfigSignal() {
    int colonIndex = networkInput.find(':');
    int semicolonIndex = networkInput.find(';');
    int newlineIndex = networkInput.find('\n');
    int newConfigNumber = std::stoi(networkInput.substr(colonIndex + 1, semicolonIndex - colonIndex - 1));
    std::string previousConfigName = networkInput.substr(semicolonIndex + 1, newlineIndex - semicolonIndex - 1);

    std::cout << "New MPTB config signal received: " << newConfigNumber << " " << std::endl;
    std::cout << "Previous config name: " << previousConfigName << std::endl;
    startNewLogfile(previousConfigName);

    if(swingUpBehavior == ReceiverConfig::CRASH_AND_SWING_UP_AT_NEW_CONFIG){
        std::cout << "Sending signal to crash and swing up again." << std::endl;
        serialActuator.Write("DOCRASHANDSWINGUP:1;\n");
    } else if(swingUpBehavior == ReceiverConfig::SWING_UP_AT_NEW_CONFIG_IF_CRASHED){
        std::cout << "Sending signal to swing up if is crashed." << std::endl;
        serialActuator.Write("DOSWINGUP:1;\n");
    }
}

/**
 * Handles the end signal of the following form: "EndSignal:previousConfigName\n"
 */
void PendulumReceiver::handleEndSignal() {
    std::cout << "End signal received." << std::endl;
    int colonIndex = networkInput.find(':');
    std::string previousConfigName = networkInput.substr(colonIndex + 1, networkInput.size() - colonIndex - 2);
    std::cout << "Previous config name: " << previousConfigName << std::endl;
    logger->setName("pendulumreceiver_" + previousConfigName);
    stop();
}

void PendulumReceiver::stop() {
    stopReceiving = true;
    receiverSocket.close();
    serialActuator.Close();
    logger->saveToFile();
}

void PendulumReceiver::startNewLogfile(std::string previousConfigName) {
    logger->setName("pendulumreceiver_" + previousConfigName);
    logger->saveToFileAsync(); // Save asynchronously to avoid blocking the main thread
    logger = new PendulumLogger();
}
