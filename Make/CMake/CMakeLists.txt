# This file is either used to build the B-Human system or to generate project files for
# either Microsoft Visual Studio or Apple Xcode. While the generated Xcode project can
# build all targets using Xcode's build system, Visual Studio only builds for Windows and
# uses CMake to cross-compile for the NAO robot. As a result, this file is used twice on
# Windows and Linux, once to either generate the project or to build for that platform
# and a second time with the symbol BUILD_NAO set to cross-compile for the NAO. As the
# Windows Subsystem for Linux is used for cross-compilation on Windows, cross-compilation
# only takes place with either Linux or macOS build tools. As Xcode is used directly for
# cross-compilation, a section of this build file basically duplicates most of the
# project generation and includes several subprojects a second time, but appends "Nao" to
# the names of the build targets. The symbol TARGET_SUFFIX is used for this.
#
# This file defines the following custom symbols to distinguish between the different
# operating systems and target architectures:
# LINUX, MACOS, WINDOWS: Boolean values for use in conditions. These describe the
#                        operating system the project is built on.
# X86, ARM:              Boolean values for use in conditions. These describe the
#                        processor architecture the code is built for.
# PLATFORM:              The name of the target OS ("Linux", "macOS", "Windows") for use
#                        in path names.
# ARCH:                  Suffix for current target PLATFORM if path names exist for
#                        different processors ("", "Arm64", "arm64").

cmake_minimum_required(VERSION 3.17)

if(BUILD_NAO)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../CMake/NaoToolchain.cmake")
endif()

if(NOT BUILD_NAO AND CMAKE_HOST_UNIX)
  set(CMAKE_CXX_COMPILER clang++)
endif()

project(B-Human LANGUAGES CXX)

if(("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL arm64 OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL aarch64) AND NOT ROSETTA)
  set(X86 false)
  set(ARM true)
  set(ARCH "Arm64")
else()
  set(X86 true)
  set(ARM false)
  set(ARCH "")
endif()

set(WINDOWS false)
set(MACOS false)
set(LINUX false)
if(APPLE)
  set(MACOS true)
  set(PLATFORM "macOS")
  if(ARM)
    set(ARCH "arm64")
  elseif(ROSETTA)
    set(CMAKE_OSX_ARCHITECTURES "x86_64")
  endif()
  add_definitions(-DMACOS)
elseif(WIN32)
  set(WINDOWS true)
  set(PLATFORM "Windows")
  add_definitions(-DWINDOWS)
else()
  set(LINUX true)
  set(PLATFORM "Linux")
  add_definitions(-DLINUX)
endif()

if(NOT BUILD_NAO)
  set(BUILD_DESKTOP ON)
endif()

if(PYTHON_ONLY)
  set(MINIMAL_PROJECT ON)
endif()

set(CMAKE_CONFIGURATION_TYPES Debug Develop Release CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Develop CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BHUMAN_PREFIX "${PROJECT_SOURCE_DIR}/../..")
set(SIMROBOT_PREFIX "${PROJECT_SOURCE_DIR}/../../Util/SimRobot")
set(OUTPUT_PREFIX "${PROJECT_SOURCE_DIR}/../..")

if(MACOS)
  set(CMAKE_SUPPRESS_REGENERATION ON)
  add_definitions(
      -DFIX_MACOS_EDIT_MENU
      -DFIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      -DFIX_MACOS_TOOLBAR_WIDGET_NOT_CLOSING)
  set(CMAKE_XCODE_ATTRIBUTE_SHARED_PRECOMPS_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/PrecompiledHeaders")
  set(CMAKE_XCODE_ATTRIBUTE_ALWAYS_SEARCH_USER_PATHS "NO")
  set(CMAKE_XCODE_ATTRIBUTE_USE_HEADERMAP "NO")
  set(CMAKE_SKIP_RPATH TRUE)
  set(BUILD_NAO ON)
  set(CMAKE_XCODE_ATTRIBUTE_CC "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/compile")
elseif(LINUX)
  set(CMAKE_CXX_COMPILER_LAUNCHER env CCACHE_CPP2=yes CCACHE_SLOPPINESS=pch_defines,time_macros "ccache")
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER CMake)
set_property(GLOBAL PROPERTY AUTOGEN_SOURCE_GROUP ".Generated Files")
if(NOT MINIMAL_PROJECT)
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT SimRobot)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_DEVELOP "")
set(CMAKE_CXX_FLAGS_RELEASE "")

set(CMAKE_EXE_LINKER_FLAGS_DEBUG "")
set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "")
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "")
set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "")
set(CMAKE_EXE_LINKER_FLAGS_DEVELOP "")
set(CMAKE_MODULE_LINKER_FLAGS_DEVELOP "")
set(CMAKE_SHARED_LINKER_FLAGS_DEVELOP "")
set(CMAKE_STATIC_LINKER_FLAGS_DEVELOP "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "")
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "")
set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "")

if(MSVC)
  add_compile_options(/W3 /GF /GS- /MP /EHsc /permissive- /Zc:preprocessor /Zc:__cplusplus /wd5105 /w34100 /w34189)
  add_definitions(-DNOMINMAX -D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS -D_CRT_SECURE_NO_DEPRECATE -D__SSSE3__)

  execute_process(COMMAND bash -c "cat /proc/cpuinfo" OUTPUT_VARIABLE _cpuinfo)
  string(FIND "${_cpuinfo}" "avx2" _has_avx2)
  string(FIND "${_cpuinfo}" "avx" _has_avx)
  if(NOT ${_has_avx2} EQUAL -1)
    add_compile_options(/arch:AVX2)
  elseif(NOT ${_has_avx} EQUAL -1)
    add_compile_options(/arch:AVX)
  endif()
else()
  add_compile_options(-pipe
      -Wall
      -Wextra
      -Wconversion
      -Wno-unknown-warning-option
      -Wno-sign-conversion
      -Wno-implicit-int-float-conversion
      -Wunreachable-code
      -Xclang
      -fno-pch-timestamp)
  if(BUILD_DESKTOP)
    if(NOT ARM AND NOT ROSETTA)
      add_compile_options(-march=native)
    endif()
    if(MACOS)
      add_compile_options(-Wno-deprecated)
      if(X86)
        add_compile_options(-mno-rtm)
      endif()
    else()
      add_link_options(-fuse-ld=mold)
    endif()
  endif()
  add_definitions(-D__extern_always_inline=inline)
endif()

set(_is_msvc "$<CXX_COMPILER_ID:MSVC>")
set(_is_clang "$<NOT:$<CXX_COMPILER_ID:MSVC>>")
set(_not_mac "$<NOT:$<PLATFORM_ID:Darwin>>")

add_library(Flags::DebugInDevelop INTERFACE IMPORTED)
target_compile_definitions(Flags::DebugInDevelop INTERFACE
    $<$<CONFIG:Release>:NDEBUG>)
target_compile_options(Flags::DebugInDevelop INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/bigobj>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/ZI>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Zi>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Release>>:/Od>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Ob2>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Oi>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Ot>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Oy>>
    $<${_is_msvc}:$<$<CONFIG:Release>:/Ox>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/GT>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Release>>:-g>>
    $<${_is_clang}:$<$<CONFIG:Release>:-O3>>
    $<${_is_clang}:$<$<CONFIG:Release>:-fomit-frame-pointer>>)
target_link_options(Flags::DebugInDevelop INTERFACE
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Release>>:/DEBUG>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/INCREMENTAL:NO>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/SAFESEH:NO>>
    $<$<AND:${_not_mac},${_is_clang}>:$<$<CONFIG:Release>:-s>>)

add_library(Flags::Default INTERFACE IMPORTED)
target_compile_definitions(Flags::Default INTERFACE
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>)
target_compile_options(Flags::Default INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/bigobj>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/ZI>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/Od>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/Ox>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/GT>>
    $<${_is_clang}:$<$<CONFIG:Debug>:-g>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-O3>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-fomit-frame-pointer>>)
target_link_options(Flags::Default INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/DEBUG>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/INCREMENTAL:NO>>
    $<$<AND:${_not_mac},${_is_clang}>:$<$<CONFIG:Release>:-s>>)

if(BUILD_DESKTOP)
  if(NOT PYTHON_ONLY)
    include("../CMake/CheckThreads.cmake")
  endif()
  if(NOT MINIMAL_PROJECT)
    include("../CMake/DeployDialog.cmake")
    if(MACOS)
      include("../CMake/Config.cmake")
    endif()
    include("${SIMROBOT_PREFIX}/Make/CMake/Box2D.cmake")
    include("${SIMROBOT_PREFIX}/Make/CMake/ODE.cmake")
    include("${SIMROBOT_PREFIX}/Make/CMake/Qt6.cmake")

    add_library(GTest::GTest STATIC IMPORTED)
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/gtest/include")
    if(WINDOWS)
      set_target_properties(GTest::GTest PROPERTIES
          IMPORTED_CONFIGURATIONS "Debug"
          IMPORTED_LOCATION_DEBUG "${BHUMAN_PREFIX}/Util/gtest/lib/${PLATFORM}${ARCH}/gtestd.lib"
          IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/gtest/lib/${PLATFORM}${ARCH}/gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/gtest/lib/${PLATFORM}${ARCH}/libgtest.a")
      if(LINUX)
        set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_OPTIONS -no-pie)
      endif()
    endif()
  endif()

  if(MACOS)
    find_library(APP_KIT_FRAMEWORK AppKit)
    find_library(IO_KIT_FRAMEWORK IOKit)
  endif()
endif()

set(EIGEN_NAN_INITIALIZATION_CONDITION "$<NOT:$<CONFIG:Release>>")
include("${SIMROBOT_PREFIX}/Make/CMake/Eigen.cmake")

add_library(GameController::GameController INTERFACE IMPORTED)
set_target_properties(GameController::GameController PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/GameController/include")

if(BUILD_NAO)
  add_library(Nao::ALSA::ALSA SHARED IMPORTED)
  set_target_properties(Nao::ALSA::ALSA PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/Buildchain/V6/lib/libasound.so.2.0.0")
elseif(LINUX)
  if(NOT MINIMAL_PROJECT)
    find_package(ALSA REQUIRED)
  endif()
endif()

if(BUILD_DESKTOP)
  add_library(snappy::snappy STATIC IMPORTED)
  set_target_properties(snappy::snappy PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/snappy/include")
  if(LINUX OR MACOS)
    set_target_properties(snappy::snappy PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/snappy/lib/${PLATFORM}${ARCH}/libsnappy.a")
  elseif(WINDOWS)
    set_target_properties(snappy::snappy PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug"
        IMPORTED_LOCATION_DEBUG "${BHUMAN_PREFIX}/Util/snappy/lib/${PLATFORM}${ARCH}/snappyd.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/snappy/lib/${PLATFORM}${ARCH}/snappy.lib")
  endif()
endif()

if(BUILD_NAO)
  add_library(Nao::libjpeg::libjpeg STATIC IMPORTED)
  set_target_properties(Nao::libjpeg::libjpeg PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/libjpeg/include")
  set_target_properties(Nao::libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/libjpeg/lib/V6/libjpeg.a")
endif()

if(BUILD_DESKTOP)
  add_library(libjpeg::libjpeg STATIC IMPORTED)
  set_target_properties(libjpeg::libjpeg PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/libjpeg/include")
  if(LINUX)
    set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/libjpeg/lib/${PLATFORM}${ARCH}/libjpeg.a")
  elseif(MACOS)
    set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/libjpeg/lib/${PLATFORM}/libjpeg.a")
  elseif(WINDOWS)
    set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/libjpeg/lib/${PLATFORM}${ARCH}/libjpeg.lib")
  endif()
endif()

if(BUILD_NAO)
  add_library(Nao::FFTW::FFTW STATIC IMPORTED)
  add_library(Nao::FFTW::FFTWF STATIC IMPORTED)
  set_target_properties(Nao::FFTW::FFTW PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
  set_target_properties(Nao::FFTW::FFTWF PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
  set_target_properties(Nao::FFTW::FFTW PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/V6/libfftw3.a")
  set_target_properties(Nao::FFTW::FFTWF PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/V6/libfftw3f.a")
endif()

if(BUILD_DESKTOP)
  if(LINUX OR MACOS)
    add_library(FFTW::FFTW STATIC IMPORTED)
    add_library(FFTW::FFTWF STATIC IMPORTED)
    set_target_properties(FFTW::FFTW PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTWF PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTW PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}${ARCH}/libfftw3.a")
    set_target_properties(FFTW::FFTWF PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}${ARCH}/libfftw3f.a")
  else(WINDOWS)
    add_library(FFTW::FFTW SHARED IMPORTED)
    add_library(FFTW::FFTWF SHARED IMPORTED)
    set_target_properties(FFTW::FFTW PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTWF PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTW PROPERTIES
        IMPORTED_IMPLIB "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}${ARCH}/libfftw3-3.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}${ARCH}/libfftw3-3.dll")
    set_target_properties(FFTW::FFTWF PROPERTIES
        IMPORTED_IMPLIB "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}${ARCH}/libfftw3f-3.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}${ARCH}/libfftw3f-3.dll")
  endif()
endif()

if(BUILD_NAO)
  add_library(Nao::HDF5::HDF5 STATIC IMPORTED)
  set_target_properties(Nao::HDF5::HDF5 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/hdf5/include")
  set_target_properties(Nao::HDF5::HDF5 PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/hdf5/lib/V6/libhdf5.a")
endif()

if(BUILD_DESKTOP)
  add_library(HDF5::HDF5 STATIC IMPORTED)
  set_target_properties(HDF5::HDF5 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/hdf5/include")
  set_target_properties(HDF5::HDF5 PROPERTIES INTERFACE_LINK_LIBRARIES $<$<PLATFORM_ID:Linux>:-ldl>)
  if(LINUX OR MACOS)
    set_target_properties(HDF5::HDF5 PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/hdf5/lib/${PLATFORM}${ARCH}/libhdf5.a")
  elseif(WINDOWS)
    set_target_properties(HDF5::HDF5 PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/hdf5/lib/${PLATFORM}${ARCH}/hdf5.lib")
  endif()
endif()

if(BUILD_NAO)
  add_library(Nao::flite::flite_cmu_us_slt STATIC IMPORTED)
  add_library(Nao::flite::flite_usenglish STATIC IMPORTED)
  add_library(Nao::flite::flite_cmulex STATIC IMPORTED)
  add_library(Nao::flite::flite STATIC IMPORTED)
  set_target_properties(Nao::flite::flite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/flite/include")
  set_target_properties(Nao::flite::flite_cmu_us_slt PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/V6/libflite_cmu_us_slt.a")
  set_target_properties(Nao::flite::flite_usenglish PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/V6/libflite_usenglish.a")
  set_target_properties(Nao::flite::flite_cmulex PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/V6/libflite_cmulex.a")
  set_target_properties(Nao::flite::flite PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/V6/libflite.a")
endif()

if(BUILD_DESKTOP AND LINUX AND NOT MINIMAL_PROJECT)
  add_library(flite::flite_cmu_us_slt STATIC IMPORTED)
  add_library(flite::flite_usenglish STATIC IMPORTED)
  add_library(flite::flite_cmulex STATIC IMPORTED)
  add_library(flite::flite STATIC IMPORTED)
  set_target_properties(flite::flite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/flite/include")
  set_target_properties(flite::flite_cmu_us_slt PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}${ARCH}/libflite_cmu_us_slt.a")
  set_target_properties(flite::flite_usenglish PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}${ARCH}/libflite_usenglish.a")
  set_target_properties(flite::flite_cmulex PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}${ARCH}/libflite_cmulex.a")
  set_target_properties(flite::flite PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}${ARCH}/libflite.a")
endif()

if(BUILD_NAO)
  add_library(Nao::ONNXRuntime::ONNXRuntime SHARED IMPORTED)
  set_target_properties(Nao::ONNXRuntime::ONNXRuntime PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/onnxruntime/include"
      IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/onnxruntime/lib/Linux/libonnxruntime.so.1.10.0")
  target_link_options(Nao::ONNXRuntime::ONNXRuntime INTERFACE -rpath /home/nao/Config)
endif()

if(BUILD_DESKTOP)
  add_library(ONNXRuntime::ONNXRuntime SHARED IMPORTED)
  set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/onnxruntime/include")
  if(LINUX)
    set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}${ARCH}/libonnxruntime.so.1.10.0")
  elseif(WINDOWS)
    set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
        IMPORTED_IMPLIB "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.dll")
    set(CONTROLLER_DYLIBS "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.dll")
  elseif(MACOS)
    set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/libonnxruntime.1.11.1.dylib")
    set(CONTROLLER_DYLIBS "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/libonnxruntime.1.11.1.dylib")
  endif()
endif()

if(BUILD_NAO)
  add_library(CompiledNN::ONNXNao INTERFACE IMPORTED)
  target_include_directories(CompiledNN::ONNXNao SYSTEM INTERFACE "${BHUMAN_PREFIX}/Util/CompiledNN2ONNX/include")
  target_link_libraries(CompiledNN::ONNXNao INTERFACE Nao::ONNXRuntime::ONNXRuntime)
endif()

if(BUILD_DESKTOP)
  add_library(CompiledNN::ONNX INTERFACE IMPORTED)
  target_include_directories(CompiledNN::ONNX SYSTEM INTERFACE "${BHUMAN_PREFIX}/Util/CompiledNN2ONNX/include")
  target_link_libraries(CompiledNN::ONNX INTERFACE ONNXRuntime::ONNXRuntime)
  if(ARM)
    target_include_directories(CompiledNN::ONNX SYSTEM INTERFACE "${BHUMAN_PREFIX}/Util/CompiledNN2ONNX/arm64/include")

    add_library(asmjit INTERFACE)
    target_include_directories(asmjit SYSTEM INTERFACE "${BHUMAN_PREFIX}/Util/CompiledNN2ONNX/arm64/include")
  endif()
endif()

add_library(voronoi::voronoi INTERFACE IMPORTED)
target_include_directories(voronoi::voronoi SYSTEM INTERFACE "${BHUMAN_PREFIX}/Util/voronoi/include")

if(X86)
  include("../CMake/asmjit.cmake")
  include("../CMake/CompiledNN.cmake")
endif()
include("../CMake/Platform.cmake")
include("../CMake/MathBase.cmake")
include("../CMake/Streaming.cmake")
include("../CMake/Math.cmake")
include("../CMake/ImageProcessing.cmake")
include("../CMake/Network.cmake")
include("../CMake/RobotParts.cmake")
include("../CMake/Debugging.cmake")
include("../CMake/Framework.cmake")
include("../CMake/OptionsAndSkills.cmake")
include("../CMake/B-Human.cmake")

if(BUILD_DESKTOP)
  include("../CMake/Python.cmake")
endif()

if(MINIMAL_PROJECT)
  set(BUILD_DESKTOP FALSE)
  set(BUILD_NAO FALSE)
  set(DISABLE_NAO TRUE)
endif()

if(BUILD_DESKTOP)
  set(SIMROBOT_CONTROLLERS SimulatedNao)
  if(MACOS)
    set(SIMROBOT_LIBRARY_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/SimRobot/$<CONFIG>/SimRobot.app/Contents/lib")
    set(ALL_ROOT_DIR "${BHUMAN_PREFIX}/Src")
    file(GLOB_RECURSE ALL_SOURCES
        "${ALL_ROOT_DIR}/Modules/*.cpp" "${ALL_ROOT_DIR}/Modules/*.h"
        "${ALL_ROOT_DIR}/Representations/*.cpp" "${ALL_ROOT_DIR}/Representations/*.h"
        "${ALL_ROOT_DIR}/Threads/*.cpp" "${ALL_ROOT_DIR}/Threads/*.h"
        "${ALL_ROOT_DIR}/Tools/*.cpp" "${ALL_ROOT_DIR}/Tools/*.h")
    add_custom_target(Src SOURCES ${ALL_SOURCES})
    source_group(TREE "${ALL_ROOT_DIR}" FILES ${ALL_SOURCES})
    set(CONTROLLER_FRAMEWORKS ${CONTROLLER_FRAMEWORKS}
        "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtSvgWidgets.framework")
  else()
    set(SIMROBOT_LIBRARY_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/SimRobot/$<CONFIG>")
    set(CONTROLLER_DYLIBS ${CONTROLLER_DYLIBS} "$<TARGET_FILE:Qt6::SvgWidgets>")
  endif()

  include("${SIMROBOT_PREFIX}/Make/CMake/SimRobot.cmake")
  include("${SIMROBOT_PREFIX}/Make/CMake/SimRobotCommon.cmake")
  include("${SIMROBOT_PREFIX}/Make/CMake/SimRobotCore2.cmake")
  include("${SIMROBOT_PREFIX}/Make/CMake/SimRobotCore2D.cmake")
  include("${SIMROBOT_PREFIX}/Make/CMake/SimRobotEditor.cmake")

  include("../CMake/qtpropertybrowser.cmake")

  if(MACOS)
    include("../CMake/AppleHelper.cmake")
  endif()

  include("../CMake/SimulatedNao.cmake")
  include("../CMake/Tests.cmake")

  set_property(TARGET SimRobot PROPERTY FOLDER Apps)
  if(MACOS)
    set_property(TARGET SimRobot SimRobotCore2 SimRobotCore2D SimRobotEditor ${SIMROBOT_CONTROLLERS} PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "B-Human Code Release")
  endif()
endif()

if(NOT MINIMAL_PROJECT)
  if(MACOS)
    set(CMAKE_OSX_ARCHITECTURES "x86_64")
    set(OS "Linux")
    set(PLATFORM "Linux")
    set(TARGET_SUFFIX "Nao")
    set(BUILD_DESKTOP FALSE)
    set(ARM_BACKUP "${ARM}")
    set(X86_BACKUP "${X86}")
    set(ARCH_BACKUP "${ARCH}")
    set(ARM false)
    set(X86 true)
    set(ARCH "")

    include("../CMake/asmjit.cmake")
    target_compile_options(asmjitNao PRIVATE
        -Wno-macro-redefined
        -Wno-implicit-int-conversion
        -Wno-enum-enum-conversion)

    include("../CMake/CompiledNN.cmake")
    include("../CMake/Platform.cmake")
    include("../CMake/MathBase.cmake")
    include("../CMake/Streaming.cmake")
    include("../CMake/Math.cmake")
    include("../CMake/ImageProcessing.cmake")
    include("../CMake/Network.cmake")
    include("../CMake/RobotParts.cmake")
    include("../CMake/Debugging.cmake")
    include("../CMake/Framework.cmake")
    include("../CMake/OptionsAndSkills.cmake")
    include("../CMake/B-Human.cmake")
    set_property(TARGET B-HumanNao PROPERTY XCODE_ATTRIBUTE_ASM_OPTIONS "-target x86_64-linux-gnu -UMACOS")

    add_library(CrossCompile INTERFACE)
    target_compile_definitions(CrossCompile INTERFACE LINUX)
    target_compile_options(CrossCompile INTERFACE
        -UMACOS
        -Qunused-arguments
        -nostdinc
        -nostdinc++
        -march=silvermont
        -mtune=silvermont
        -mrdrnd
        -target x86_64-linux-gnu)
    target_include_directories(CrossCompile SYSTEM INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/include/c++/12"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/include/x86_64-linux-gnu/c++/12"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/include/c++/12/backward"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/include/x86_64-linux-gnu"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/include")
    target_link_directories(CrossCompile INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/lib")

    target_link_libraries(asmjitNao PUBLIC CrossCompile)
    target_link_libraries(CompiledNNNao PUBLIC CrossCompile)
    target_link_libraries(PlatformNao PUBLIC CrossCompile)
    target_link_libraries(MathBaseNao PUBLIC CrossCompile)
    target_link_libraries(StreamingNao PUBLIC CrossCompile)
    target_link_libraries(MathNao PUBLIC CrossCompile)
    target_link_libraries(ImageProcessingNao PUBLIC CrossCompile)
    target_link_libraries(NetworkNao PUBLIC CrossCompile)
    target_link_libraries(RobotPartsNao PUBLIC CrossCompile)
    target_link_libraries(DebuggingNao PUBLIC CrossCompile)
    target_link_libraries(FrameworkNao PUBLIC CrossCompile)
    target_link_libraries(OptionsAndSkillsNao PUBLIC CrossCompile)
    target_link_libraries(B-HumanNao PUBLIC CrossCompile)

    set_property(TARGET
        asmjitNao
        CompiledNNNao
        PROPERTY XCODE_ATTRIBUTE_LIBTOOL "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/libtool")
    set_property(TARGET
        B-Human
        B-Human-Optimized
        Debugging
        Framework
        ImageProcessing
        MathBase
        Math
        Network
        Platform
        RobotParts
        OptionsAndSkills
        Streaming

        B-HumanNao
        DebuggingNao
        FrameworkNao
        ImageProcessingNao
        MathBaseNao
        MathNao
        NetworkNao
        PlatformNao
        RobotPartsNao
        OptionsAndSkillsNao
        StreamingNao
        PROPERTY XCODE_ATTRIBUTE_LIBTOOL "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/ignoretool")
  endif()

  include("../CMake/Nao.cmake")
  if(MACOS)
    set(CMAKE_OSX_ARCHITECTURES "")
    set(TARGET_SUFFIX "")
    set(BUILD_DESKTOP TRUE)
    set(ARM "${ARM_BACKUP}")
    set(X86 "${X86_BACKUP}")
    set(ARCH "${ARCH_BACKUP}")
    add_dependencies(DeployDialog Nao)
  endif()
endif()

if(BUILD_DESKTOP AND NOT MINIMAL_PROJECT)
  include("../CMake/deploy.cmake")
endif()

source_group(".PCH" REGULAR_EXPRESSION ".*[ch]xx$")
source_group(".Visualizers" REGULAR_EXPRESSION ".*natvis$")
source_group(".CMake Rules" REGULAR_EXPRESSION ".*rule$")
