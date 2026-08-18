CXX = g++
CXXFLAGS = -Wall -std=c++17 -O2 -I../../core

CORE_SRC = ../../core/WooDriveSdk.cpp
CORE_DEP = ../../core/WooDriveSdk.cpp ../../core/WooDriveSdk.h

check_id: check_id.cpp $(CORE_DEP)
	$(CXX) $(CXXFLAGS) -o $@ check_id.cpp $(CORE_SRC)

clean:
	rm -f check_id
