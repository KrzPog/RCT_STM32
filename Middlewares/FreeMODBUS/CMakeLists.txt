cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

set (MX_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
)

set (MB_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/ascii
    ${CMAKE_CURRENT_LIST_DIR}/functions
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/port
    ${CMAKE_CURRENT_LIST_DIR}/rtu
    ${CMAKE_CURRENT_LIST_DIR}/tcp
)

set (MB_SRC
    ${CMAKE_CURRENT_LIST_DIR}/ascii/mbascii.c
    ${CMAKE_CURRENT_LIST_DIR}/functions/mbfunccoils.c
    # ${CMAKE_CURRENT_LIST_DIR}/functions/mbfuncdiag.c
    ${CMAKE_CURRENT_LIST_DIR}/functions/mbfuncdisc.c
    ${CMAKE_CURRENT_LIST_DIR}/functions/mbfuncholding.c
    ${CMAKE_CURRENT_LIST_DIR}/functions/mbfuncinput.c
    ${CMAKE_CURRENT_LIST_DIR}/functions/mbfuncother.c
    ${CMAKE_CURRENT_LIST_DIR}/functions/mbutils.c
    ${CMAKE_CURRENT_LIST_DIR}/port/portcritical.c
    ${CMAKE_CURRENT_LIST_DIR}/port/portevent.c
    ${CMAKE_CURRENT_LIST_DIR}/port/portserial.c
    ${CMAKE_CURRENT_LIST_DIR}/port/porttimer.c
    ${CMAKE_CURRENT_LIST_DIR}/rtu/mbcrc.c
    ${CMAKE_CURRENT_LIST_DIR}/rtu/mbrtu.c
    ${CMAKE_CURRENT_LIST_DIR}/tcp/mbtcp.c
    ${CMAKE_CURRENT_LIST_DIR}/mb.c
)

add_library(FreeMODBUS STATIC ${MB_SRC})
target_include_directories(FreeMODBUS PUBLIC ${MX_INCLUDE_DIRS} ${MB_INCLUDE_DIRS})
target_link_libraries(FreeMODBUS PUBLIC stm32cubemx)