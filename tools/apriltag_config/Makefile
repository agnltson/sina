# ==============================
# Configuration
# ==============================
PROJECT      := apriltag_config
CC           := gcc
CSTD         := -std=c23
WARNINGS     := -Wall
WARNINGS     += -Wextra
WARNINGS     += -Werror
WARNINGS     += -Wpedantic
WARNINGS     += -Wshadow
CFLAGS       := $(CSTD) $(WARNINGS) -g
INCLUDES     := -Iinclude
SRC_DIR      := src
BUILD_DIR    := build
CLANG_FORMAT := clang-format
CLANG_TIDY   := clang-tidy

# ==============================
# External libs
# ==============================
PKG_CONFIG   := pkg-config
FFMPEG_LIBS  := libavformat libavcodec libswscale libavutil

APRILTAG_CFLAGS := $(shell $(PKG_CONFIG) --cflags apriltag 2>/dev/null || echo "-I/usr/local/include/apriltag")
APRILTAG_LIBS    := $(shell $(PKG_CONFIG) --libs apriltag 2>/dev/null || echo "-L/usr/local/lib -lapriltag")

EXT_CFLAGS   := $(shell $(PKG_CONFIG) --cflags $(FFMPEG_LIBS)) $(APRILTAG_CFLAGS)
EXT_LIBS     := $(shell $(PKG_CONFIG) --libs $(FFMPEG_LIBS)) $(APRILTAG_LIBS) -lm -lpthread

INCLUDES     += $(EXT_CFLAGS)

# ==============================
# Sources
# ==============================
SRC_FILES    := $(shell find $(SRC_DIR) -name "*.c")
TEST_FILES   := $(shell find $(TEST_DIR) -name "*.c")
SRC_NO_MAIN := $(filter-out $(SRC_DIR)/main.c,$(SRC_FILES))
OBJ_NO_MAIN := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC_NO_MAIN))
OBJ_FILES    := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC_FILES))
TARGET       := $(BUILD_DIR)/$(PROJECT)

# ==============================
# Default target
# ==============================
.PHONY: all
all: $(TARGET)

# ==============================
# Build main binary
# ==============================
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
$(TARGET): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) -o $@ $(EXT_LIBS)

# ==============================
# Format
# ==============================
.PHONY: format
format:
	@echo "Formatting..."
	@find $(SRC_DIR) include $(TEST_DIR) -name "*.c" -o -name "*.h" | \
	xargs $(CLANG_FORMAT) -i

# ==============================
# Tidy
# ==============================
.PHONY: tidy
tidy:
	@echo "Running clang-tidy..."
	@for file in $(SRC_FILES); do \
		$(CLANG_TIDY) $$file -- $(INCLUDES) $(CFLAGS) || exit 1; \
	done

# ==============================
# Clean
# ==============================
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

# ==============================
# Rebuild
# ==============================
.PHONY: re
re: clean all
