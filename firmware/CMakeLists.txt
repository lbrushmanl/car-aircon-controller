cmake_minimum_required(VERSION 3.28.1)
project(car-aircon-controller C)

# MCU and frequency
set(MCU atmega88pa)
set(F_CPU 8000000UL)
set(BAUD 9600UL)

add_compile_options(
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -DBAUD=${BAUD}
    -Os -g -std=gnu99
    -Wall -Werror -Wextra -Wpadded
    -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wmissing-prototypes
    -ffunction-sections -fdata-sections
    -Wstack-usage=1000 
    -fstack-usage
)

# Global link options
add_link_options(
    -mmcu=${MCU}
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections

)

# Include drivers
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/submodules/avr-drivers)
include_directories(${CMAKE_SOURCE_DIR}/submodules/avr-drivers/utils)

# Create executable
add_executable(${PROJECT_NAME}.elf)

target_sources(${PROJECT_NAME}.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/message_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/persistent_data.c
)

target_include_directories(${PROJECT_NAME}.elf
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    task_scheduler
    debouncer
    hysteresis
    filters
    adc
    uart
    twi
    ssd1306-driver
)

# Generate HEX file
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.eep
    COMMENT "Generating HEX file"
)
