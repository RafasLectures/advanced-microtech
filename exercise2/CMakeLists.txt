
file(GLOB_RECURSE SOURCES
    ./*.cpp
    )

add_executable(exercise2  ${SOURCES})

target_link_libraries(exercise2 PRIVATE AdvancedMicrotech::Libs)

target_include_directories(exercise2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
