CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wpedantic -O2
INC      := -Isrc
LDFLAGS  := -pthread
# make MOSQUITTO=1  -> real MQTT backend (needs libmosquitto-dev)
ifeq ($(MOSQUITTO),1)
CXXFLAGS += -DUSE_MOSQUITTO
LDFLAGS  += -lmosquitto
endif

all: tank_sim tank_tests
tank_sim: src/main.cpp src/*.hpp
	$(CXX) $(CXXFLAGS) $(INC) $< -o $@ $(LDFLAGS)
tank_tests: tests/test_main.cpp src/*.hpp
	$(CXX) $(CXXFLAGS) $(INC) $< -o $@ $(LDFLAGS)
test: tank_tests
	./tank_tests
evidence: tank_sim
	./scripts/run_all.sh
sanitize:
	$(CXX) -std=c++17 -g -O1 -fsanitize=address,undefined $(INC) tests/test_main.cpp -o tank_tests_asan $(LDFLAGS) && ./tank_tests_asan
clean:
	rm -f tank_sim tank_tests tank_tests_asan
.PHONY: all test evidence sanitize clean
