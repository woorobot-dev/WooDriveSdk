CXX = g++
CXXFLAGS = -Wall -std=c++17 -O2 -I../core

CORE_SRC = ../core/WooDriveSdk.cpp
CORE_DEP = ../core/WooDriveSdk.cpp ../core/WooDriveSdk.h

EXAMPLES_DIR = examples

TARGETS = \
Example01_BasicCheck \
Example02_ReadStatus \
Example03_AutoMotorSetup \
Example04_Speed \
Example05_Position

all: $(TARGETS)

Example01_BasicCheck: $(EXAMPLES_DIR)/Example01_BasicCheck.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ $(EXAMPLES_DIR)/Example01_BasicCheck.cpp $(CORE_SRC)

Example02_ReadStatus: $(EXAMPLES_DIR)/Example02_ReadStatus.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ $(EXAMPLES_DIR)/Example02_ReadStatus.cpp $(CORE_SRC)

Example03_AutoMotorSetup: $(EXAMPLES_DIR)/Example03_AutoMotorSetup.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ $(EXAMPLES_DIR)/Example03_AutoMotorSetup.cpp $(CORE_SRC)

Example04_Speed: $(EXAMPLES_DIR)/Example04_Speed.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ $(EXAMPLES_DIR)/Example04_Speed.cpp $(CORE_SRC)

Example05_Position: $(EXAMPLES_DIR)/Example05_Position.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ $(EXAMPLES_DIR)/Example05_Position.cpp $(CORE_SRC)

clean:
	rm -f $(TARGETS)
