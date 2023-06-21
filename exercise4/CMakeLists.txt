
file(GLOB_RECURSE SOURCES
    ./*.cpp
    )

add_executable(exercise4  ${SOURCES})

target_link_libraries(exercise4 PRIVATE AdvancedMicrotech::Libs)

target_include_directories(exercise4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
