
//
// Created by david on 10.01.23.
//

#include "PendulumSender.h"
#include "../Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h"
#include <sstream>
#include <iostream>
#include <chrono>

const int RESTRICT_LOGGING_TO_MS = 50;

PendulumSender::PendulumSender(PriorityDeterminer* priorityDeterminer, std::string serialDeviceName,
                               std::string receiverHost, int receiverPort, int teensyHistorySize,
                               std::vector<int> teensySamplingPeriods, std::function<void()> regularCallback, std::string logFilePrefix)
                               : logger(logFilePrefix), regularCallback(regularCallback) {
    this->serialDeviceName = serialDeviceName;
    receiverAddress = inet_address(receiverHost, receiverPort);

    serialSensor = SerialPort(serialDeviceName, BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialSensor.SetTimeout(-1);

    this->priorityDeterminer = priorityDeterminer;
    this->teensyHistorySize = teensyHistorySize;
    this->teensySamplingPeriods = teensySamplingPeriods;
}

void PendulumSender::start() {
    serialSensor.Open();

    // Wait for sender to be ready:
    std::cout << "Waiting for sender Teensy to send READY signal." << std::endl;
    serialSensor.Read(serialInputBuffer);
    while (serialInputBuffer.rfind("READY", 0) != 0) {
        serialInputBuffer.clear();
        serialSensor.Read(serialInputBuffer);
    }

    // Send Teensy history size:
    std::cout << "Sending Teensy history size: " << teensyHistorySize << std::endl;
    // Create string of form "H:historySize;period1;period2;period3;...\n"
    std::string teensyInitParams = "H:" + std::to_string(teensyHistorySize) + ";";
    for (int period : teensySamplingPeriods) {
        teensyInitParams += std::to_string(period) + ";";
    }
    teensyInitParams += "\n";
    serialSensor.Write(teensyInitParams);

    // Wait for first serial values to arrive, before going into main loop:
    std::cout << "Waiting for first sensor value" << std::endl;
    serialSensor.Read(serialInputBuffer);
    serialInputBuffer.clear();
    std::cout << serialInputBuffer << std::endl;

    startTime = timeSinceEpochMillisec();

    std::cout << "StartTime: " << startTime << std::endl;
    while (!stopSending) {
        serialInputBuffer.clear();
        serialSensor.Read(serialInputBuffer);

        if (serialInputBuffer.rfind("FB:", 0) == 0) {
            handleSenderFeedback();
            if(!pendulumStarted){
                pendulumStarted = true;

                priorityDeterminer->resetState(); // Reset priority determiner when pendulum starts balancing
            }

        } else if(serialInputBuffer.rfind("S:", 0) == 0){
            // Sometimes the serial interface sends multiple samples at once (separated by '\n').
            // We need to split them up and send them individually over the network:
            std::string singleSample;
            std::istringstream sampleStream(serialInputBuffer);
            while (std::getline(sampleStream, singleSample, '\n')) {
                singleSample += '\n';
                sendPacket(singleSample);
            }
        }

        if(regularCallback != nullptr){
            regularCallback();
        }
    }
}

void PendulumSender::handleSenderFeedback() {
    logger.logSenderFeedback(serialInputBuffer);
    feedbackPacketsCount++;

    if(feedbackPacketsCount % 10 == 0){
        std::cout << serialInputBuffer << std::endl;
    }
}

void PendulumSender::stop() {
    stopSending = true;
    serialSensor.Close();
    logger.saveToFile();
}

void PendulumSender::sendPacket(std::string payload) {
    // Pad payload width '#' to 32 bytes and store result in paddedPayload
    std::string paddedPayload = payload;

    paddedPayload.append(32 - payload.size(), '#');


    priorityDeterminer->reportPacketReadyToSend(paddedPayload.size());

    int priority = priorityDeterminer->getPriority();

    // Drop packets if priority is BE.
    if (priority == 7) {
        std::cout << "PendulumSender: Packet dropped because of BE Priority" << std::endl;
        return;
    }


    senderSocket.set_option(SOL_SOCKET, SO_PRIORITY, priority);
    senderSocket.send_to(paddedPayload, receiverAddress);

    packetCount++;
    bytesSentTotal += paddedPayload.size();

    
    // Limit logging rate
    uint64_t currentTime = timeSinceEpochMillisec();
    if(currentTime - lastLogTime >= RESTRICT_LOGGING_TO_MS){
        lastLogTime = currentTime;
        logger.log(packetCount, bytesSentTotal, payload, priorityDeterminer->getSchedulingInfoEntry());
    }

    // Calculate current pole angle
    // Payload: S:encoderValue;transmissionPeriodMillis;sequenceNumber;currentTime
    //std::cout << "Payload: " << payload;
    size_t delimPos = payload.find(';');
    std::string currentValue = payload.substr(2,delimPos);
    int encoderValue = std::stoi(currentValue);
    const double DEG_PER_ESTEP = 360.0 / 2400.0;
    double poleAngle = DEG_PER_ESTEP * (encoderValue-1200+10); //1200 is half, 10 is compensation. (mod(encoderPos - encoderOrigin - encoderPPRhalf, encoderPPR) - encoderPPRhalf);

    // Calculate squared poleAngle Error AVG
    uint64_t passedTime = currentTime - startTime;
    // Ignore first 60s
    if(passedTime > 60000){ 
        int runNr = passedTime / 60000;
        // Wrap up previous run if runNr changed
        if(currentRunNr < runNr){
            allRunsAVG = (allRunsAVG * allRuns + currentRunAVG) / (allRuns + 1);
            allRuns++;
            currentRunNr = runNr;
        }
        currentRunAVG = (currentRunAVG * currentRunValues + poleAngle * poleAngle) / (currentRunValues + 1);
        currentRunValues++;
    }


    if (packetCount % 10 == 0) {
        /*std::cout << "PendulumSender: "
                  << "Sent " << packetCount << " packets. "
                  << priorityDeterminer->getDebugInfoString()
                  << " Payload: " << payload
                  << std::endl;*/
        std::cout << priorityDeterminer->getDebugInfoString() << ", CurrentAngle: " << poleAngle << ", currentAVG: " << currentRunAVG << ", allAVG: " << allRunsAVG << std::endl;
    }
}

uint64_t PendulumSender::timeSinceEpochMillisec(){
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void PendulumSender::swapPriorityDeterminer(PriorityDeterminer *newPriorityDeterminer, std::string logFilePrefix) {
    // Save logs of old priorityDeterminer to file:
    logger.saveToFile();

    // Swap priorityDeterminer
    priorityDeterminer = newPriorityDeterminer;

    // Reset statistics
    startTime = timeSinceEpochMillisec();
    packetCount = 0;
    bytesSentTotal = 0;
    feedbackPacketsCount = 0;
    currentRunAVG = 0;
    currentRunValues = 0;
    currentRunNr = 1;
    allRunsAVG = 0;
    allRuns = 0;

    // new logger for new priorityDeterminer
    logger = PendulumLogger(logFilePrefix);
}

/**
 * Sends a signal to the receiver to change the MPTB config to the given number.
 *
 * The payload of the signal is "NewConfig:number\n" where number is the parameter of this function encoded as a string.
 * The signal is sent with the highest priority (0).
 *
 * @throws std::runtime_error if senderSocket is not open
 * @param number
 */
void PendulumSender::sendNewMptbConfigSignal(int number) {
    if(!senderSocket.is_open()){
        throw std::runtime_error("PendulumSender: Cannot send new MPTB config signal because senderSocket is not open.");
    }

    std::string payload = "NewConfig:" + std::to_string(number) + "\n";
    senderSocket.set_option(SOL_SOCKET, SO_PRIORITY, 0);
    senderSocket.send_to(payload, receiverAddress);
    std::cout << "PendulumSender: Sent new MPTB config signal: " << payload << std::endl;
}

/**
 * Sends a signal to the receiver to end the current run.
 *
 * The payload of the signal is "EndSignal\n".
 * The signal is sent with the highest priority (0).
 *
 * @throws std::runtime_error if senderSocket is not open
 */
void PendulumSender::sendEndSignal() {
    if(!senderSocket.is_open()){
        throw std::runtime_error("PendulumSender: Cannot send End signal because senderSocket is not open.");
    }

    std::string payload = "EndSignal\n";
    senderSocket.set_option(SOL_SOCKET, SO_PRIORITY, 0);
    senderSocket.send_to(payload, receiverAddress);

    std::cout << "PendulumSender: Sent End signal: " << payload << std::endl;
}

