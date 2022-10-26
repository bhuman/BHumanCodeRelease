set(NAO_ROOT_DIR "${BHUMAN_PREFIX}/Src/Apps/Nao")
set(NAO_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/Nao/$<CONFIG>")

set(NAO_SOURCES
    "${NAO_ROOT_DIR}/Main.cpp")

if(BUILD_NAO)
  if(NOT APPLE AND NOT (${PLATFORM} STREQUAL Linux AND ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang))
    message(ERROR "The target Nao has to be built for Linux and with clang.")
  endif()

  add_executable(Nao ${NAO_SOURCES})
  set_property(TARGET Nao PROPERTY RUNTIME_OUTPUT_DIRECTORY "${NAO_OUTPUT_DIR}")
  set_property(TARGET Nao PROPERTY RUNTIME_OUTPUT_NAME bhuman)
  set_property(TARGET Nao PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
  target_link_libraries(Nao PRIVATE -lpthread-2.31)
  target_link_libraries(Nao PRIVATE B-Human${TARGET_SUFFIX})
  target_link_libraries(Nao PRIVATE Platform${TARGET_SUFFIX})
  target_link_libraries(Nao PRIVATE Math${TARGET_SUFFIX})
  target_link_libraries(Nao PRIVATE Streaming${TARGET_SUFFIX})
  target_link_libraries(Nao PRIVATE RobotParts${TARGET_SUFFIX})
  target_link_libraries(Nao PRIVATE Framework${TARGET_SUFFIX})

  target_compile_options(Nao PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  if(APPLE)
    set_property(TARGET Nao PROPERTY XCODE_ATTRIBUTE_LDPLUSPLUS_x86_64 "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/link")
    set_property(TARGET Nao PROPERTY XCODE_GENERATE_SCHEME ON)
    set_property(TARGET Nao PROPERTY XCODE_SCHEME_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/../Common/deployDialog")
    set_property(TARGET Nao PROPERTY XCODE_SCHEME_ARGUMENTS "Debug")
    set_property(TARGET Nao PROPERTY FOLDER Apps)
    source_group(TREE "${NAO_ROOT_DIR}" FILES ${NAO_SOURCES})
  endif()

  target_link_libraries(Nao PRIVATE Flags::Default)
elseif(${PLATFORM} STREQUAL Windows)
  set(CMAKE_MSVCIDE_RUN_PATH "%systemroot%\\Sysnative")
  add_custom_target(Nao ALL bash -c "stdbuf -e0 -oL cmake --build . --target Nao | stdbuf -e0 -oL sed 's@^/mnt/\\([a-z]\\)@\\U\\1:@'"
      WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build/Linux/CMake/$<CONFIG>"
      SOURCES ${NAO_SOURCES})
  set_property(TARGET Nao PROPERTY FOLDER Apps)

  source_group(TREE "${NAO_ROOT_DIR}" FILES ${NAO_SOURCES})
else()
  include(ExternalProject)
  ExternalProject_Add(Nao
      SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
      CMAKE_ARGS -DCMAKE_BUILD_TYPE=$<CONFIG> -DBUILD_NAO=ON
      BUILD_ALWAYS 1
      USES_TERMINAL_BUILD ON
      INSTALL_COMMAND "")
  set_property(TARGET Nao PROPERTY FOLDER "")
endif()
