cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_examples C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# iterate over a list of examples by name
set(EXAMPLES_LIST
    gettingStarted
    # acknowledgementPayloads
    # streamingData
    # manualAcknowledgements
    # multiceiverDemo
    # interruptConfigure
    # scanner
)

foreach(example ${EXAMPLES_LIST})
    # make a target
    add_executable(${example}
        ${example}.c
        ../RF24.c 
        ../RF24.h 
        ../RF24_config.h 
        ../nRF24L01.h 
        pico_port.c
        pico_port.h
        defaultPins.h)

    # link the necessary libs to the target
    target_link_libraries(${example} PUBLIC
        pico_stdlib
        hardware_spi
        hardware_gpio
    )

    # specify USB port as default serial communication's interface (not UART RX/TX pins)
    pico_enable_stdio_usb(${example} 1)
    pico_enable_stdio_uart(${example} 0)

    # create map/bin/hex file etc.
    pico_add_extra_outputs(${example})
endforeach()
