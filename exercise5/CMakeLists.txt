
file(GLOB_RECURSE SOURCES
    ./*.cpp
    )

add_executable(exercise5  ${SOURCES})

target_link_libraries(exercise5 PRIVATE AdvancedMicrotech::Libs)

target_include_directories(exercise5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
