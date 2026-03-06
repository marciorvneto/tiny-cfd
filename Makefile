OUT_DIR := ./out
TARGET := $(OUT_DIR)/main

CPPFLAGS := -I./vendor/tinyla -I./vendor/raylib/build/raylib/include
LDFLAGS := -L./vendor/raylib/build/raylib
LDLIBS := -lraylib -lm -lX11
CFLAGS := -g

all: $(TARGET)

$(TARGET): main.c | $(OUT_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

clean:
	rm -rf $(OUT_DIR)

.PHONY: all clean
