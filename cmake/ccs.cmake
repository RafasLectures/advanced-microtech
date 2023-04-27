# In order to be able to cross-compile, we first need to know what is the toolchain path.
# If the compiler toolchain path is externally defined, then all the other Code Composer specific
# directories can be derived.
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
