OUT_DIR := ./out
all: $(OUT_DIR)/main

TINY_LA := ./vendor/tinyla
CFLAGS := -lm -g -I$(TINY_LA)

$(OUT_DIR)/main: main.c | $(OUT_DIR)
	$(CC) -o $@ $< $(CFLAGS)

$(OUT_DIR):
	@mkdir -p $(OUT_DIR)

clean:
	@rm -rf $(OUT_DIR)
