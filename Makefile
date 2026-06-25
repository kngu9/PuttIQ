CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Imain -Itest

TESTS := test_vec3 test_preprocess test_features test_detector test_orientation test_result

.PHONY: test replay clean preview
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

# ---------------------------------------------------------------------------
# HOST-ONLY LVGL -> PNG preview (NOT part of the device build or `make test`).
# Compiles LVGL v9.5 into a static archive (cached) + the harness, then runs it
# to emit build/preview_test.png. Building LVGL is slow on first run; reruns are
# fast because liblvgl.a is cached.
# ---------------------------------------------------------------------------
LVGL_DIR := /Users/khanh/Documents/Arduino/libraries/lvgl
# NOTE: -Itest MUST precede -I$(LVGL_DIR)/.. so our test/lv_conf.h is picked up
# instead of the device's /Users/khanh/Documents/Arduino/libraries/lv_conf.h
# (which lives in $(LVGL_DIR)/.. and disables fonts we need here).
LVGL_INC := -Itest -Imain -I$(LVGL_DIR) -I$(LVGL_DIR)/.. -DLV_CONF_INCLUDE_SIMPLE
LVGL_CC  := gcc
LVGL_CFLAGS := -std=c11 -O2 $(LVGL_INC)

LVGL_SRCS := $(shell find $(LVGL_DIR)/src -name '*.c')
LVGL_OBJS := $(patsubst $(LVGL_DIR)/%.c,build/lvgl/%.o,$(LVGL_SRCS))

build/lvgl/%.o: $(LVGL_DIR)/%.c test/lv_conf.h
	@mkdir -p $(dir $@)
	$(LVGL_CC) $(LVGL_CFLAGS) -c $< -o $@

build/liblvgl.a: $(LVGL_OBJS)
	ar rcs $@ $(LVGL_OBJS)

build/ui_preview: test/ui_preview.cpp test/lv_conf.h test/stb_image_write.h build/liblvgl.a
	@mkdir -p build
	$(CXX) -std=c++17 -O2 $(LVGL_INC) test/ui_preview.cpp build/liblvgl.a -o $@

preview: build/ui_preview
	@mkdir -p build
	./build/ui_preview
