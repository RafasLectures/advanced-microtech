if(NOT DEFINED CGT_TOOLCHAIN_DIR)
  if (NOT DEFINED CCS_BASE_DIR)
    if (NOT DEFINED TI_BASE_DIR)
      if (WIN32)
        set(TI_BASE_DIR "C:/ti")
      elseif(APPLE)
        set(TI_BASE_DIR "/Applications/ti")
      elseif(UNIX)
        message(FATAL_ERROR "Please set the base directory where CCS is installed. E.g /opt/ti")
        # I'm not sure where that is, so I just left it like that. Later I can add it
        #set(TI_BASE_DIR "/opt/ti")
      endif()
    endif()
    # Finds the root where CCS is installed. If there are multiple versions.
    # It will get the first from the list.
    file(GLOB CCS_BASE_DIRS RELATIVE ${TI_BASE_DIR}  "${TI_BASE_DIR}/ccs*" )
    if(NOT CCS_BASE_DIRS)
      message(FATAL_ERROR "Did not find CCS directory")
    endif()
    list(GET CCS_BASE_DIRS 0 CCS_BASE_DIR)
    set(CCS_BASE_DIR ${TI_BASE_DIR}/${CCS_BASE_DIR}/ccs)
  else()
    # Derive TI_BASE_DIR information later
  endif()
else()
  # Derive CCS_BASE_DIR
  set(TOOLS_DIR ${CGT_TOOLCHAIN_DIR}) # Only initialization
  while((NOT TOOLS_ROOT_NAME STREQUAL "tools") AND (NOT TOOLS_DIR STREQUAL ""))
    cmake_path(GET TOOLS_DIR PARENT_PATH TOOLS_DIR)
    cmake_path(GET TOOLS_DIR FILENAME TOOLS_ROOT_NAME)
  endwhile()
  # One extra time, since we stopped in "tools"
  cmake_path(GET TOOLS_DIR PARENT_PATH CCS_BASE_DIR)

  # Derive TI_BASE_DIR
  set(TI_BASE_DIR ${CCS_BASE_DIR})
  while((NOT TI_BASE_ROOT_NAME STREQUAL "ti") AND (NOT TI_BASE_DIR STREQUAL ""))
    cmake_path(GET TI_BASE_DIR PARENT_PATH TI_BASE_DIR)
    cmake_path(GET TI_BASE_DIR FILENAME TI_BASE_ROOT_NAME)
  endwhile()
endif()

message(STATUS "TI_BASE_DIR: "  ${TI_BASE_DIR})
message(STATUS "CCS_BASE_DIR: "  ${CCS_BASE_DIR})

set(CCS_COMPILERS_PATH ${CCS_BASE_DIR}/tools/compiler)

# ALL OF THE ABOVE SHOULD BE AN include(${PROJECT_SOURCE_DIR}/cmake/ccs.cmake)

if(NOT DEFINED CGT_TOOLCHAIN_DIR)
  # Looks for the MSP430 compilers stored.
  file(GLOB MSP430_COMPILERS RELATIVE ${CCS_COMPILERS_PATH}  "${CCS_COMPILERS_PATH}/ti-cgt-msp430*" )
  if(NOT MSP430_COMPILERS)
    message(FATAL_ERROR "Did not find MSP430 toolchain installed in CCS")
  endif()
  # Get the last from the list, since in theory the last will be the later version due to
  # the alphabetical ordering of the list
  list(GET MSP430_COMPILERS -1 CGT_TOOLCHAIN_DIR)
  set(CGT_TOOLCHAIN_DIR ${CCS_COMPILERS_PATH}/${CGT_TOOLCHAIN_DIR})
endif()
message(STATUS "CGT_TOOLCHAIN_DIR: "  ${CGT_TOOLCHAIN_DIR})

set(CCS_BASE_MSP430 ${CCS_BASE_DIR}/ccs_base/msp430)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR MSP430G2553)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# set target environment
set(CMAKE_FIND_ROOT_PATH ${CGT_TOOLCHAIN_DIR})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# toolchain paths
find_program(TI_GCC             NAMES   cl430    PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_CXX             NAMES   cl430    PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_AS              NAMES   cl430    PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_AR              NAMES   ar430    PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_OBJCOPY         NAMES   ofd430   PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_OBJDUMP         NAMES   hex430   PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_SIZE            NAMES   size430  PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)
find_program(TI_LD              NAMES   cl430    PATHS  ${CGT_TOOLCHAIN_DIR}/bin    NO_DEFAULT_PATH)

# set executables settings
set(CMAKE_C_COMPILER    ${TI_GCC})
set(CMAKE_CXX_COMPILER  ${TI_CXX})
set(AS                  ${TI_AS})
set(AR                  ${TI_AR})
set(OBJCOPY             ${TI_OBJCOPY})
set(OBJDUMP             ${TI_OBJDUMP})
set(SIZE                ${TI_SIZE})
set(LD                  ${TI_LD})

# set default include directory
include_directories(
    ${CGT_TOOLCHAIN_DIR}/include
    ${CGT_TOOLCHAIN_DIR}/include/libcxx
    ${CCS_BASE_MSP430}/include
)

link_directories(
    ${CGT_TOOLCHAIN_DIR}/lib
    ${CCS_BASE_MSP430}/include # Local where default .cmd files are
)
#-g
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --c++14 -vmsp --use_hw_mpy=none --advice:power='all' -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --c++14 -vmsp --use_hw_mpy=none --advice:power='all' -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile")

set(MSP_COMPILER 1)
add_definitions(-D__MSP430G2553__)
# definition

#-vmsp --use_hw_mpy=none --advice:power="all" --define=__MSP430G2553__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number -z -m"Exercise_2_AndrioliBauer.map" --heap_size=80 --stack_size=80 -i"/Applications/ti/ccs1200/ccs/ccs_base/msp430/include" -i"/Applications/ti/ccs1200/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/lib" -i"/Applications/ti/ccs1200/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="Exercise_2_AndrioliBauer_linkInfo.xml" --use_hw_mpy=none --rom_model -o "Exercise_2_AndrioliBauer.out" "./common/ShiftRegister.obj" "./exercise3/exercise3.obj" "../lnk_msp430g2553.cmd"  -llibc.a
