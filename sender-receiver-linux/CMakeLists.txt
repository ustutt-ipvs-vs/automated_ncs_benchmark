cmake_minimum_required(VERSION 3.22)
project(sender-receiver-linux)

set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(cross_traffic
        CrossTraffic/CrossTrafficSender.cpp
        CrossTraffic/CrossTrafficSender.h
        CrossTraffic/CrossTrafficSenderOrchestrator.cpp
        CrossTraffic/CrossTrafficSenderOrchestrator.h
        CrossTraffic/CrossTrafficMain.cpp
        Scheduling/TokenBucket.hpp
        Scheduling/TokenBucket.cpp
        Logging/Logger.cpp
        Logging/Logger.h
        Logging/LogEntries/LogEntry.cpp
        Logging/LogEntries/LogEntry.h
        nlohmann/json.hpp
        Logging/CrossTrafficLogger.cpp
        Logging/CrossTrafficLogger.h
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h
        Scheduling/PriorityDeterminer.h
        Scheduling/PriorityDeterminer.cpp
        Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.h
        Scheduling/ConstantPriority.cpp
        Scheduling/ConstantPriority.h
        Scheduling/MultiPriorityTokenBucket.cpp
        Scheduling/MultiPriorityTokenBucket.h
        )

add_executable(pendulum_sender
        PendulumSenderSide/PendulumSender.cpp
        PendulumSenderSide/PendulumSender.h
        Logging/PendulumLogger.cpp
        Logging/PendulumLogger.h
        Logging/LogEntries/PendulumLogEntry.cpp
        Logging/LogEntries/PendulumLogEntry.h
        Logging/Logger.cpp
        Logging/Logger.h
        Logging/LogEntries/LogEntry.cpp
        Logging/LogEntries/LogEntry.h
        nlohmann/json.hpp
        Scheduling/TokenBucket.hpp
        Scheduling/TokenBucket.cpp
        PendulumSenderSide/PendulumSenderMain.cpp
        Logging/LogEntries/SchedulingInfoEntries/SchedulingInfoEntry.h
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h
        Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.h
        Logging/LogEntries/ActuatorLogEntry.cpp
        Logging/LogEntries/ActuatorLogEntry.h
        Scheduling/ConstantPriority.cpp
        Scheduling/ConstantPriority.h
        Logging/LogEntries/PauseLogEntry.h
        Logging/LogEntries/PauseLogEntry.cpp
        Logging/LogEntries/SenderFeedbackLogEntry.cpp
        Logging/LogEntries/SenderFeedbackLogEntry.h
        Scheduling/PriorityDeterminer.h
        Scheduling/PriorityDeterminer.cpp
        Scheduling/MultiPriorityTokenBucket.cpp
        Scheduling/MultiPriorityTokenBucket.h
        PendulumSenderSide/SenderConfig.cpp
        PendulumSenderSide/SenderConfig.h
        SerialPortScan/TeensyPortDetector.cpp
        SerialPortScan/TeensyPortDetector.h
        PendulumSenderSide/MPTBSubConfig.cpp
        PendulumSenderSide/MPTBSubConfig.h
        PendulumSenderSide/SenderMultiConfig.h
        PendulumSenderSide/SenderMultiConfig.cpp)

add_executable(pendulum_receiver
        PendulumReceiverSide/PendulumReceiverMain.cpp
        PendulumReceiverSide/PendulumReceiver.cpp
        PendulumReceiverSide/PendulumReceiver.h
        Logging/PendulumLogger.cpp
        Logging/PendulumLogger.h
        Logging/LogEntries/PendulumLogEntry.cpp
        Logging/LogEntries/PendulumLogEntry.h
        Logging/Logger.cpp
        Logging/Logger.h
        Logging/LogEntries/LogEntry.cpp
        Logging/LogEntries/LogEntry.h
        nlohmann/json.hpp
        Logging/LogEntries/ActuatorLogEntry.cpp
        Logging/LogEntries/ActuatorLogEntry.h
        Logging/LogEntries/PauseLogEntry.h
        Logging/LogEntries/PauseLogEntry.cpp
        Logging/LogEntries/SenderFeedbackLogEntry.cpp
        Logging/LogEntries/SenderFeedbackLogEntry.h
        Scheduling/PriorityDeterminer.h
        Scheduling/PriorityDeterminer.cpp
        Scheduling/MultiPriorityTokenBucket.cpp
        Scheduling/MultiPriorityTokenBucket.h
        Scheduling/TokenBucket.hpp
        Scheduling/TokenBucket.cpp
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h
        SerialPortScan/TeensyPortDetector.cpp
        SerialPortScan/TeensyPortDetector.h
        PendulumReceiverSide/ReceiverConfig.cpp
        PendulumReceiverSide/ReceiverConfig.h
        )


target_link_libraries(cross_traffic sockpp)

target_link_libraries(pendulum_sender sockpp)
target_link_libraries(pendulum_sender CppLinuxSerial)

target_link_libraries(pendulum_receiver sockpp)
target_link_libraries(pendulum_receiver CppLinuxSerial)

