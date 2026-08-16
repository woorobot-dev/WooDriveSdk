cmake_minimum_required(VERSION 3.16)
project(woodrive_python LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_woodrive
    src/bindings.cpp
    ../core/WooDriveSdk.cpp
)
target_include_directories(_woodrive PRIVATE ../core)

if(WIN32)
  target_sources(_woodrive PRIVATE ../core/WindowsSerialTransport.cpp)
endif()

install(TARGETS _woodrive DESTINATION woodrive)
