
file(GLOB_RECURSE SOURCES
    ./*.cpp
    ${PROJECT_SOURCE_DIR}/exercise5/audiorecorder/*.cpp
    )

add_executable(audiorecorder-example  ${SOURCES})

target_link_libraries(audiorecorder-example
        PRIVATE AdvancedMicrotech::Libs)

target_include_directories(audiorecorder-example PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${PROJECT_SOURCE_DIR})
