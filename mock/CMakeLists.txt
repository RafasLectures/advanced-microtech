
file(GLOB_RECURSE SOURCES
    ./*.hpp
    ./*.cpp
    )

add_library(msp430Mock STATIC ${SOURCES})

target_include_directories(msp430Mock PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

