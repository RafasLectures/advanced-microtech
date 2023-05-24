
add_subdirectory(common)

set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/i2c.cpp)

add_library(libs ${SOURCES})
add_library(AdvancedMicrotech::Libs ALIAS libs)
target_link_libraries(libs PUBLIC AdvancedMicrotech::Common)

target_include_directories(libs PUBLIC ${PROJECT_SOURCE_DIR})


