cmake_minimum_required(VERSION 3.0.0)
project(default-test-project VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
set(CMAKE_CXX_STANDARD 20)

include_directories(inc)
include_directories(impl)

add_executable(default-test-project
    main.cpp
    src/cpu.cpp
    src/alu.cpp
    src/reg.cpp
    src/int.cpp
    src/timer.cpp
    src/ccp.cpp
    src/adc.cpp
    src/port.cpp
    src/environment.cpp)

if(MSVC)
    target_compile_options(default-test-project PRIVATE /W4 /WX)
else()
    target_compile_options(default-test-project PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(CMAKE_CXX_FLAGS "-O0")