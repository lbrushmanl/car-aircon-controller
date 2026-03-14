.PHONY: build clean size format flash_eeprom init

BUILD_DIR := build
TARGET := car-aircon-controller
BAUD := 9600
MCU := atmega88pa
PROGRAMMER_TYPE := USBasp

init:
	@git submodule update --init

build:
	@if [ ! -f $(BUILD_DIR)/build.ninja ]; then \
		cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=avr-gcc-toolchain.cmake -B $(BUILD_DIR); \
	fi
	cmake --build $(BUILD_DIR)

flash: build
	avrdude -c $(PROGRAMMER_TYPE) -p $(MCU) -U flash:w:$(BUILD_DIR)/$(TARGET).hex:i

clean:
	@rm -rf $(BUILD_DIR)

size:  build
	@avr-size -C --mcu=$(MCU) $(BUILD_DIR)/$(TARGET).elf

flash_eeprom:
	avrdude -c $(PROGRAMMER_TYPE) -p $(MCU) -U eeprom:w:$(BUILD_DIR)/$(TARGET).eep:i

# Root directory of your source files
SRC_DIR := .

# Find all .c and .h files recursively
SRC_FILES := $(shell find $(SRC_DIR) -name '*.c' -o -name '*.h')

# Format target
format:
	@echo "Formatting source files..."
	@clang-format -style=file -i $(SRC_FILES)
	@echo "Done."