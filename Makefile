CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Imain -Itest

TESTS := test_vec3 test_preprocess test_features test_detector test_orientation test_result

.PHONY: test replay clean
test: $(TESTS:%=build/%)
	@set -e; for t in $(TESTS); do echo "=== $$t ==="; ./build/$$t; done

build/%: test/%.cpp $(wildcard main/*.cpp main/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(wildcard main/putt_*.cpp) $< -o $@

replay: build/replay
build/replay: test/replay.cpp $(wildcard main/*.cpp main/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(wildcard main/putt_*.cpp) test/replay.cpp -o build/replay

clean:
	rm -rf build
