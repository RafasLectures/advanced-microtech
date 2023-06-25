
file(GLOB_RECURSE SOURCES
    ./*.cpp
    )

add_executable(stringbuilderexample  ${SOURCES})

target_link_libraries(stringbuilderexample PRIVATE AdvancedMicrotech::Libs)

target_include_directories(stringbuilderexample PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
