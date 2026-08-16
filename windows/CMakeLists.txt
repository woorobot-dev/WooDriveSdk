cmake_minimum_required(VERSION 3.16)
project(WooDriveWindowsCheck LANGUAGES CXX)

add_executable(WooDriveBasicCheck
    BasicCheck.cpp
    ../core/WindowsSerialTransport.cpp
    ../core/WooDriveSdk.cpp
)
target_compile_features(WooDriveBasicCheck PRIVATE cxx_std_17)
target_compile_options(WooDriveBasicCheck PRIVATE -Wall -Wextra)

add_executable(WooDriveLowSpeedRun
    LowSpeedRun.cpp
    ../core/WindowsSerialTransport.cpp
    ../core/WooDriveSdk.cpp
)
target_compile_features(WooDriveLowSpeedRun PRIVATE cxx_std_17)
target_compile_options(WooDriveLowSpeedRun PRIVATE -Wall -Wextra)

add_executable(WooDriveContinuous20Rpm
    Continuous20Rpm.cpp
    ../core/WindowsSerialTransport.cpp
    ../core/WooDriveSdk.cpp
)
target_compile_features(WooDriveContinuous20Rpm PRIVATE cxx_std_17)
target_compile_options(WooDriveContinuous20Rpm PRIVATE -Wall -Wextra)

add_executable(WooDrivePosition360
    Position360.cpp
    ../core/WindowsSerialTransport.cpp
    ../core/WooDriveSdk.cpp
)
target_compile_features(WooDrivePosition360 PRIVATE cxx_std_17)
target_compile_options(WooDrivePosition360 PRIVATE -Wall -Wextra)
