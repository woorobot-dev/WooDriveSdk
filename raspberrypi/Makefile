CXX = g++
CXXFLAGS = -Wall -std=c++17 -O2 -I../core

CORE_SRC = ../core/WooDriveSdk.cpp
CORE_DEP = ../core/WooDriveSdk.cpp ../core/WooDriveSdk.h

TARGETS = \
Example01_BasicCheck \
Example02_ReadStatus \
Example03_AutoMotorSetup \
Example04_Speed \
Example05_Position

all: $(TARGETS)

Example01_BasicCheck: Example01_BasicCheck.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ Example01_BasicCheck.cpp $(CORE_SRC)

Example02_ReadStatus: Example02_ReadStatus.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ Example02_ReadStatus.cpp $(CORE_SRC)

Example03_AutoMotorSetup: Example03_AutoMotorSetup.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ Example03_AutoMotorSetup.cpp $(CORE_SRC)

Example04_Speed: Example04_Speed.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ Example04_Speed.cpp $(CORE_SRC)

Example05_Position: Example05_Position.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ Example05_Position.cpp $(CORE_SRC)

clean:
	rm -f $(TARGETS)
