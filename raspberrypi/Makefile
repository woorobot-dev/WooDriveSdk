CXX = g++
CXXFLAGS = -Wall -std=c++17 -O2

TARGETS = \
Example01_BasicCheck \
Example02_ReadStatus \
Example03_AutoMotorSetup \
Example04_Speed \
Example05_Position

all: $(TARGETS)

Example01_BasicCheck: Example01_BasicCheck.cpp WooDriveSdk.cpp WooDriveSdk.h
	$(CXX) $(CXXFLAGS) -o $@ Example01_BasicCheck.cpp WooDriveSdk.cpp

Example02_ReadStatus: Example02_ReadStatus.cpp WooDriveSdk.cpp WooDriveSdk.h
	$(CXX) $(CXXFLAGS) -o $@ Example02_ReadStatus.cpp WooDriveSdk.cpp

Example03_AutoMotorSetup: Example03_AutoMotorSetup.cpp WooDriveSdk.cpp WooDriveSdk.h
	$(CXX) $(CXXFLAGS) -o $@ Example03_AutoMotorSetup.cpp WooDriveSdk.cpp

Example04_Speed: Example04_Speed.cpp WooDriveSdk.cpp WooDriveSdk.h
	$(CXX) $(CXXFLAGS) -o $@ Example04_Speed.cpp WooDriveSdk.cpp

Example05_Position: Example05_Position.cpp WooDriveSdk.cpp WooDriveSdk.h
	$(CXX) $(CXXFLAGS) -o $@ Example05_Position.cpp WooDriveSdk.cpp

clean:
	rm -f $(TARGETS)
