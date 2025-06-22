cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

set (SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/Src/bldc_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/bldc_interface_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/buffer.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/crc.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/packet.c
)

add_library(VESC_UART_Interface STATIC ${SRC_FILES})
target_include_directories(VESC_UART_Interface PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Inc)