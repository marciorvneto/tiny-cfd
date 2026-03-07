OUT_DIR := ./out
OUT_BROWSER_DIR := ./browser-viz
TARGET := $(OUT_DIR)/main
TARGET_BROWSER := $(OUT_BROWSER_DIR)/browser.js # Must be .js for Emscripten glue code!

# Native Flags
CPPFLAGS := -I./vendor/tinyla -I./vendor/raylib/build/raylib/include
LDFLAGS := -L./vendor/raylib/build/raylib
LDLIBS := -lraylib -lm -lX11
CFLAGS := -g

# WASM Flags
EMCC_FLAGS := -O3 -s ALLOW_MEMORY_GROWTH=1 -s EXPORTED_RUNTIME_METHODS='["ccall", "cwrap"]'

all: $(TARGET) $(TARGET_BROWSER)

$(TARGET): main.c | $(OUT_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

$(TARGET_BROWSER): browser.c | $(OUT_BROWSER_DIR)
	emcc $(EMCC_FLAGS) $< -o $@ -lm

$(OUT_DIR):
	mkdir -p $(OUT_DIR)
$(OUT_BROWSER_DIR):
	mkdir -p $(OUT_BROWSER_DIR)

clean:
	rm -rf $(OUT_DIR)
	rm -rf $(OUT_BROWSER_DIR)

.PHONY: all clean
