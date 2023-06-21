
file(GLOB_RECURSE SOURCES
    ./*.cpp
    )

add_executable(exercise3  ${SOURCES})

target_link_libraries(exercise3 PRIVATE AdvancedMicrotech::Libs)

target_include_directories(exercise3 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
