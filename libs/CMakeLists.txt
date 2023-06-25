
add_subdirectory(common)

file(GLOB_RECURSE SOURCES
        ./*.cpp
        ./*.c)

add_library(libs ${SOURCES})
add_library(AdvancedMicrotech::Libs ALIAS libs)
target_link_libraries(libs PUBLIC AdvancedMicrotech::Common)

target_include_directories(libs PUBLIC ${PROJECT_SOURCE_DIR})


