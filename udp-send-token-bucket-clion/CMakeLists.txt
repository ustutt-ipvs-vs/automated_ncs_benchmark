cmake_minimum_required(VERSION 3.22)
project(udp_send_token_bucket_clion)

set(CMAKE_CXX_STANDARD 17)

add_executable(udp_send_token_bucket_clion udp_mass_send_token_bucket.cpp TokenBucket.cpp TokenBucketPrioTest.hpp TokenBucketPrioTest.cpp)
add_executable(cross_traffic_host CrossTrafficHost.cpp TokenBucket.cpp TokenBucketPrioTest.hpp TokenBucketPrioTest.cpp)

add_executable(cross_traffic
        CrossTraffic/CrossTrafficSender.cpp
        CrossTraffic/CrossTrafficSender.h
        CrossTraffic/CrossTrafficSenderOrchestrator.cpp
        CrossTraffic/CrossTrafficSenderOrchestrator.h
        CrossTraffic/CrossTrafficMain.cpp
        TokenBucket.cpp TokenBucketPrioTest.hpp
        TokenBucketPrioTest.cpp
        Logging/Logger.cpp
        Logging/Logger.h
        Logging/LogEntries/LogEntry.cpp
        Logging/LogEntries/LogEntry.h
        nlohmann/json.hpp
        Logging/CrossTrafficLogger.cpp
        Logging/CrossTrafficLogger.h
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h
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
        TokenBucket.cpp
        TokenBucketPrioTest.hpp
        TokenBucketPrioTest.cpp
        PendulumSenderSide/PendulumSenderMain.cpp
        Logging/LogEntries/SchedulingInfoEntries/SchedulingInfoEntry.h
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.cpp
        Logging/LogEntries/SchedulingInfoEntries/TokenBucketInfoEntry.h
        )

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
        )

target_link_libraries(udp_send_token_bucket_clion sockpp)
target_link_libraries(udp_send_token_bucket_clion CppLinuxSerial)

target_link_libraries(cross_traffic_host sockpp)

target_link_libraries(cross_traffic sockpp)

target_link_libraries(pendulum_sender sockpp)
target_link_libraries(pendulum_sender CppLinuxSerial)

target_link_libraries(pendulum_receiver sockpp)
target_link_libraries(pendulum_receiver CppLinuxSerial)