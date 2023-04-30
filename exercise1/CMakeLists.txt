
file(GLOB_RECURSE SOURCES
    ./*.cpp
    )

add_executable(exercise1  ${SOURCES})

target_link_libraries(exercise1 PRIVATE AdvancedMicrotech::Libs)

target_include_directories(exercise1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
