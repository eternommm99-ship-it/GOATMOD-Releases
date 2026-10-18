cmake_minimum_required(VERSION 3.16)
project(goatmod_physics012 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(goatmod_physics012 STATIC
    src/physics012_geometry.cpp
    src/physics013_motion.cpp
    src/physics014_contact.cpp
)

target_include_directories(goatmod_physics012 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (MSVC)
    target_compile_options(goatmod_physics012 PRIVATE /W4)
else()
    target_compile_options(goatmod_physics012 PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(goatmod_physics012_test
    tests/physics012_geometry_test.cpp
)
target_link_libraries(goatmod_physics012_test PRIVATE goatmod_physics012)

add_executable(goatmod_physics013_motion_test
    tests/physics013_motion_test.cpp
)
target_link_libraries(goatmod_physics013_motion_test PRIVATE goatmod_physics012)

add_executable(goatmod_physics014_contact_test
    tests/physics014_contact_test.cpp
)
target_link_libraries(goatmod_physics014_contact_test PRIVATE goatmod_physics012)

enable_testing()
add_test(NAME goatmod_physics012_geometry COMMAND goatmod_physics012_test)
add_test(NAME goatmod_physics013_motion COMMAND goatmod_physics013_motion_test)
add_test(NAME goatmod_physics014_contact COMMAND goatmod_physics014_contact_test)
