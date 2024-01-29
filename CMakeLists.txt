cmake_minimum_required(VERSION 3.0.0)
project(pic18-emu VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
set(CMAKE_CXX_STANDARD 20)

include_directories(inc)
include_directories(impl)

add_executable(pic18-emu
    main.cpp
    src/cpu.cpp
    src/alu.cpp
    src/bank.cpp
    src/tbl.cpp
    src/config.cpp
    src/int.cpp
    src/timer.cpp
    src/ccp.cpp
    src/adc.cpp
    src/port.cpp
    src/eusart.cpp
    src/environment.cpp)

if(MSVC)
    target_compile_options(pic18-emu PRIVATE /W4 /WX)
else()
    target_compile_options(pic18-emu PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(CMAKE_CXX_FLAGS "-O0")