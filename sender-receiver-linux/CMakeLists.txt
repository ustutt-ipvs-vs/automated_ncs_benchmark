cmake_minimum_required(VERSION 3.22)
project(sender-receiver-linux)

set(CMAKE_CXX_STANDARD 17)

add_executable(cross_traffic
        CrossTraffic/CrossTrafficSender.cpp
        CrossTraffic/CrossTrafficSender.h
        CrossTraffic/CrossTrafficSenderOrchestrator.cpp
        CrossTraffic/CrossTrafficSenderOrchestrator.h
        CrossTraffic/CrossTrafficMain.cpp
        Scheduling/TokenBucket.cpp Scheduling/TokenBucketPrioTest.hpp
        Scheduling/TokenBucketPrioTest.cpp
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
        Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/ConstantPrioInfoEntry.h
        Scheduling/ConstantPriority.cpp
        Scheduling/ConstantPriority.h)

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
        Scheduling/TokenBucket.cpp
        Scheduling/TokenBucketPrioTest.hpp
        Scheduling/TokenBucketPrioTest.cpp
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
        Logging/LogEntries/PauseLogEntry.cpp)

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
        Logging/LogEntries/PauseLogEntry.cpp)


target_link_libraries(cross_traffic sockpp)

target_link_libraries(pendulum_sender sockpp)
target_link_libraries(pendulum_sender CppLinuxSerial)

target_link_libraries(pendulum_receiver sockpp)
target_link_libraries(pendulum_receiver CppLinuxSerial)