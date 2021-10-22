set(NAO_ROOT_DIR "${BHUMAN_PREFIX}/Src")
set(NAO_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/Nao/$<CONFIG>")

file(GLOB_RECURSE NAO_SOURCES
    "${NAO_ROOT_DIR}/Modules/*.cpp" "${NAO_ROOT_DIR}/Modules/*.h")
file(GLOB NAO_SOURCES_ADDITIONAL
    "${NAO_ROOT_DIR}/Platform/*.cpp" "${NAO_ROOT_DIR}/Platform/*.h")
list(APPEND NAO_SOURCES ${NAO_SOURCES_ADDITIONAL})
file(GLOB_RECURSE NAO_SOURCES_ADDITIONAL
    "${NAO_ROOT_DIR}/Representations/*.cpp" "${NAO_ROOT_DIR}/Representations/*.h"
    "${NAO_ROOT_DIR}/Threads/*.cpp" "${NAO_ROOT_DIR}/Threads/*.h"
    "${NAO_ROOT_DIR}/Tools/*.cpp" "${NAO_ROOT_DIR}/Tools/*.h"
    "${NAO_ROOT_DIR}/Platform/Nao/*.cpp" "${NAO_ROOT_DIR}/Platform/Nao/*.h")
list(APPEND NAO_SOURCES ${NAO_SOURCES_ADDITIONAL})

if(BUILD_NAO)
  if(NOT APPLE AND NOT (${PLATFORM} STREQUAL Linux AND ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang))
    message(ERROR "The target Nao has to be built for Linux and with clang.")
  endif()

  add_executable(Nao ${NAO_SOURCES})
  set_property(TARGET Nao PROPERTY RUNTIME_OUTPUT_DIRECTORY "${NAO_OUTPUT_DIR}")
  set_property(TARGET Nao PROPERTY RUNTIME_OUTPUT_NAME bhuman)
  set_property(TARGET Nao PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
  target_include_directories(Nao PRIVATE "${NAO_ROOT_DIR}")
  target_link_libraries(Nao PRIVATE -lrt-2.31)
  target_link_libraries(Nao PRIVATE -lpthread-2.31)
  target_link_libraries(Nao PRIVATE Eigen::Eigen)
  target_link_libraries(Nao PRIVATE Nao::FFTW::FFTW Nao::FFTW::FFTWF)
  target_link_libraries(Nao PRIVATE Nao::libjpeg::libjpeg)
  target_link_libraries(Nao PRIVATE Nao::flite::flite_cmu_us_slt Nao::flite::flite_usenglish Nao::flite::flite_cmulex Nao::flite::flite)
  target_link_libraries(Nao PRIVATE Nao::ALSA::ALSA)
  target_link_libraries(Nao PRIVATE GameController::GameController)
  if(APPLE)
    target_link_libraries(Nao PRIVATE asmjitNao)
    target_link_libraries(Nao PRIVATE CompiledNNNao)
  else()
    target_link_libraries(Nao PRIVATE asmjit)
    target_link_libraries(Nao PRIVATE CompiledNN)
  endif()
  target_link_libraries(Nao PRIVATE -ldl-2.31)

  target_compile_definitions(Nao PRIVATE TARGET_ROBOT __STRICT_ANSI__ CONFIGURATION=$<CONFIG>)

  target_compile_options(Nao PRIVATE -Wno-switch)
  target_compile_options(Nao PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_compile_options(Nao PRIVATE $<$<CONFIG:Release>:-Wno-unused>)
  if(APPLE)
    set_property(TARGET Nao PROPERTY XCODE_ATTRIBUTE_LDPLUSPLUS_x86_64 "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/link")
    set_property(TARGET Nao PROPERTY XCODE_GENERATE_SCHEME ON)
    set_property(TARGET Nao PROPERTY XCODE_SCHEME_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/../Common/deployDialog")
    set_property(TARGET Nao PROPERTY XCODE_SCHEME_ARGUMENTS "Debug")
    source_group(TREE "${NAO_ROOT_DIR}" FILES ${NAO_SOURCES})
  endif()

  target_link_libraries(Nao PRIVATE Flags::Default)
  target_precompile_headers(Nao PRIVATE "${NAO_ROOT_DIR}/Tools/Precompiled/BHumanPch.h")
elseif(${PLATFORM} STREQUAL Windows)
  set(CMAKE_MSVCIDE_RUN_PATH "%systemroot%\\Sysnative")
  add_custom_target(Nao ALL bash -c "stdbuf -e0 -oL cmake --build . --target Nao | stdbuf -e0 -oL sed 's@^/mnt/\\([a-z]\\)@\\U\\1:@'"
      WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build/Linux/CMake/$<CONFIG>"
      SOURCES ${NAO_SOURCES})

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
