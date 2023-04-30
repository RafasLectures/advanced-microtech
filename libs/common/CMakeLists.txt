
file(GLOB_RECURSE SOURCES
        ./*.cpp
        )

add_library(advanced-microtech-common STATIC ${SOURCES})
add_library(AdvancedMicrotech::Common ALIAS advanced-microtech-common)

target_include_directories(advanced-microtech-common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if (MSP_COMPILER)
    target_link_directories(advanced-microtech-common PUBLIC ${PROJECT_SOURCE_DIR})
    target_link_libraries(advanced-microtech-common
            INTERFACE -llnk_msp430g2553.cmd
            INTERFACE -llibc.a
            )
else ()
    target_link_libraries(advanced-microtech-common PUBLIC msp430Mock)
endif ()

