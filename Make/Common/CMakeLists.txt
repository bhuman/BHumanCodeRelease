cmake_minimum_required(VERSION 3.16)

if(BUILD_NAO)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../CMake/NaoToolchain.cmake")
elseif(CMAKE_HOST_UNIX)
  set(CMAKE_CXX_COMPILER clang++)
endif()

if(NOT BUILD_NAO)
  set(BUILD_DESKTOP ON)
endif()

if(APPLE)
  project(B-Human-temp LANGUAGES CXX)
else()
  project(B-Human LANGUAGES CXX)
endif()

set(CMAKE_CONFIGURATION_TYPES Debug Develop Release CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Develop CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BHUMAN_PREFIX "${PROJECT_SOURCE_DIR}/../..")
set(SIMROBOT_PREFIX "${PROJECT_SOURCE_DIR}/../../Util/SimRobot")
set(OUTPUT_PREFIX "${PROJECT_SOURCE_DIR}/../..")

if(APPLE)
  set(CMAKE_SUPPRESS_REGENERATION ON)
  add_definitions(
      -DMACOS
      -DFIX_MACOS_DOCKED_WIDGETS_DRAG_BUG
      -DFIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      -DFIX_MACOS_NO_CENTER_IN_PINCH_GESTURE_BUG
      -DFIX_MACOS_UNDOCKED_WIDGETS_DURING_CLOSE_BUG
      -DFIX_MACOS_UNDOCKED_WIDGETS_DISAPPEAR_WHEN_DOCKED_BUG)
  if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL arm64)
    if(ROSETTA)
      set(PLATFORM "macOS")
      set(CMAKE_OSX_ARCHITECTURES "x86_64")
    else()
      set(PLATFORM "macOSarm64")
    endif()
  else()
    set(PLATFORM "macOS")
  endif()
  set(OS "macOS")

  set(CMAKE_XCODE_ATTRIBUTE_SHARED_PRECOMPS_DIR "${OUTPUT_PREFIX}/Build/${OS}/PrecompiledHeaders")
  set(CMAKE_XCODE_ATTRIBUTE_ALWAYS_SEARCH_USER_PATHS "NO")
elseif(WIN32)
  add_definitions(-DWINDOWS)
  set(PLATFORM "Windows")
  set(OS "${PLATFORM}")
elseif(UNIX)
  add_definitions(-DLINUX)
  set(PLATFORM "Linux")
  set(OS "${PLATFORM}")
endif()

if(CMAKE_HOST_APPLE)
  set(BUILD_NAO ON)
  set(CMAKE_XCODE_ATTRIBUTE_CC "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/compile")
elseif(CMAKE_HOST_WIN32 AND BUILD_NAO)
  set(CMAKE_CXX_COMPILER_LAUNCHER env CCACHE_CPP2=yes CCACHE_SLOPPINESS=pch_defines,time_macros ccache)
elseif(CMAKE_HOST_UNIX AND NOT CMAKE_HOST_WIN32)
  set(CMAKE_CXX_COMPILER_LAUNCHER env CCACHE_CPP2=yes CCACHE_SLOPPINESS=pch_defines,time_macros "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/Linux/bin/ccache")
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER CMake)
set_property(GLOBAL PROPERTY AUTOGEN_SOURCE_GROUP "Generated Files")
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY VS_STARTUP_PROJECT SimRobot)

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
  add_compile_options(/W3 /GF /GS- /MP /Zc:wchar_t- /EHsc /permissive- /Zc:preprocessor /wd5105)
  add_definitions(-DNOMINMAX -D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS -D_CRT_SECURE_NO_DEPRECATE -D__SSSE3__)

  execute_process(COMMAND bash -c "cat /proc/cpuinfo" OUTPUT_VARIABLE _cpuinfo)
  string(FIND "${_cpuinfo}" "avx2" _has_avx2)
  if(NOT ${_has_avx2} EQUAL -1)
    add_compile_options(/arch:AVX2)
  else()
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
    if(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL arm64) # Option does not work on Apple M1, but "The Internet" says it also causes trouble on multiple other arm platforms.
      add_compile_options(-march=native)
    endif()
    if(APPLE)
      add_compile_options(-Wno-deprecated)
      if(NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL arm64)
        add_compile_options(-mno-rtm)
      endif()
    else()
      add_link_options(-fuse-ld=lld)
    endif()
  endif()
  add_definitions(-D__extern_always_inline=inline)
endif()

set(_is_msvc "$<CXX_COMPILER_ID:MSVC>")
set(_is_clang "$<NOT:$<CXX_COMPILER_ID:MSVC>>")
set(_not_mac "$<NOT:$<PLATFORM_ID:Darwin>>")

add_library(Flags::ForDevelop INTERFACE IMPORTED)
target_compile_definitions(Flags::ForDevelop INTERFACE
    $<$<CONFIG:Release>:NDEBUG>)
target_compile_options(Flags::ForDevelop INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/bigobj>>
    $<${_is_msvc}:$<$<CONFIG:Debug>:/ZI>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Zi>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Release>>:/Od>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Ob2>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Oi>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Ot>>
    $<${_is_msvc}:$<$<CONFIG:Develop>:/Oy>>
    $<${_is_msvc}:$<$<CONFIG:Release>:/Ox>>
    $<${_is_msvc}:$<$<CONFIG:Release>:/wd4189>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/GT>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Release>>:-g>>
    $<${_is_clang}:$<$<CONFIG:Release>:-O3>>
    $<${_is_clang}:$<$<CONFIG:Release>:-fomit-frame-pointer>>
    $<${_is_clang}:$<$<CONFIG:Release>:-Wno-unused-parameter>>
    $<${_is_clang}:$<$<CONFIG:Release>:-Wno-unused-variable>>
    $<${_is_clang}:$<$<CONFIG:Release>:-Wno-unused-lambda-capture>>)
target_link_options(Flags::ForDevelop INTERFACE
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
    $<${_is_msvc}:$<$<CONFIG:Release>:/wd4189>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/GT>>
    $<${_is_clang}:$<$<CONFIG:Debug>:-g>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-O3>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-fomit-frame-pointer>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-Wno-unused-parameter>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-Wno-unused-variable>>
    $<${_is_clang}:$<$<NOT:$<CONFIG:Debug>>:-Wno-unused-lambda-capture>>)
target_link_options(Flags::Default INTERFACE
    $<${_is_msvc}:$<$<CONFIG:Debug>:/DEBUG>>
    $<${_is_msvc}:$<$<NOT:$<CONFIG:Debug>>:/INCREMENTAL:NO>>
    $<$<AND:${_not_mac},${_is_clang}>:$<$<CONFIG:Release>:-s>>)

if(BUILD_DESKTOP)
  include("${SIMROBOT_PREFIX}/Make/Common/Box2D.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/ODE.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/OpenGL.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/Qt5.cmake")

  if(APPLE)
    find_library(APP_KIT_FRAMEWORK AppKit)
    find_library(CORE_SERVICES_FRAMEWORK CoreServices)
    find_library(IO_KIT_FRAMEWORK IOKit)

    if(${PLATFORM} STREQUAL macOSarm64)
      add_library(ONNXRuntime::ONNXRuntime SHARED IMPORTED)
      set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
          IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.framework/onnxruntime"
          INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.framework;${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.framework/Headers")
    endif()
  endif()

  add_library(GTest::GTest STATIC IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/gtest/include")
  if((${PLATFORM} STREQUAL Linux) OR APPLE)
    set_target_properties(GTest::GTest PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/gtest/lib/${PLATFORM}/libgtest.a")
  elseif(${PLATFORM} STREQUAL Windows)
    set_target_properties(GTest::GTest PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug"
        IMPORTED_LOCATION_DEBUG "${BHUMAN_PREFIX}/Util/gtest/lib/${PLATFORM}/gtestd.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/gtest/lib/${PLATFORM}/gtest.lib")
  endif()
endif()

set(EIGEN_NAN_INITIALIZATION_CONDITION "$<NOT:$<CONFIG:Release>>")
include("${SIMROBOT_PREFIX}/Make/Common/Eigen.cmake")

add_library(GameController::GameController INTERFACE IMPORTED)
set_target_properties(GameController::GameController PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/GameController/include")

if(BUILD_NAO)
  add_library(Nao::ALSA::ALSA SHARED IMPORTED)
  set_target_properties(Nao::ALSA::ALSA PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/Buildchain/V6/gcc/lib/libasound.so")
elseif(${PLATFORM} STREQUAL "Linux")
  find_package(ALSA REQUIRED)
endif()

if(BUILD_DESKTOP)
  add_library(snappy::snappy STATIC IMPORTED)
  set_target_properties(snappy::snappy PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/snappy/include")
  if((${PLATFORM} STREQUAL Linux) OR APPLE)
    set_target_properties(snappy::snappy PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/snappy/lib/${PLATFORM}/libsnappy.a")
  elseif(${PLATFORM} STREQUAL Windows)
    set_target_properties(snappy::snappy PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug"
        IMPORTED_LOCATION_DEBUG "${BHUMAN_PREFIX}/Util/snappy/lib/${PLATFORM}/snappyd.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/snappy/lib/${PLATFORM}/snappy.lib")
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
  if((${PLATFORM} STREQUAL Linux) OR APPLE)
    set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/libjpeg/lib/${PLATFORM}/libjpeg.a")
  elseif(${PLATFORM} STREQUAL Windows)
    set_target_properties(libjpeg::libjpeg PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/libjpeg/lib/${PLATFORM}/libjpeg.lib")
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
  if((${PLATFORM} STREQUAL Linux) OR APPLE)
    add_library(FFTW::FFTW STATIC IMPORTED)
    add_library(FFTW::FFTWF STATIC IMPORTED)
    set_target_properties(FFTW::FFTW PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTWF PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTW PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}/libfftw3.a")
    set_target_properties(FFTW::FFTWF PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}/libfftw3f.a")
  else(${PLATFORM} STREQUAL Windows)
    add_library(FFTW::FFTW SHARED IMPORTED)
    add_library(FFTW::FFTWF SHARED IMPORTED)
    set_target_properties(FFTW::FFTW PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTWF PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/fftw-3.3")
    set_target_properties(FFTW::FFTW PROPERTIES
        IMPORTED_IMPLIB "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}/libfftw3-3.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}/libfftw3-3.dll")
    set_target_properties(FFTW::FFTWF PROPERTIES
        IMPORTED_IMPLIB "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}/libfftw3f-3.lib"
        IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/fftw-3.3/${PLATFORM}/libfftw3f-3.dll")
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
  if((${PLATFORM} STREQUAL Linux) OR APPLE)
    set_target_properties(HDF5::HDF5 PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/hdf5/lib/${PLATFORM}/libhdf5.a")
  elseif(${PLATFORM} STREQUAL Windows)
    set_target_properties(HDF5::HDF5 PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/hdf5/lib/${PLATFORM}/hdf5.lib")
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

if(BUILD_DESKTOP AND (${PLATFORM} STREQUAL Linux))
  add_library(flite::flite_cmu_us_slt STATIC IMPORTED)
  add_library(flite::flite_usenglish STATIC IMPORTED)
  add_library(flite::flite_cmulex STATIC IMPORTED)
  add_library(flite::flite STATIC IMPORTED)
  set_target_properties(flite::flite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BHUMAN_PREFIX}/Util/flite/include")
  set_target_properties(flite::flite_cmu_us_slt PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}/libflite_cmu_us_slt.a")
  set_target_properties(flite::flite_usenglish PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}/libflite_usenglish.a")
  set_target_properties(flite::flite_cmulex PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}/libflite_cmulex.a")
  set_target_properties(flite::flite PROPERTIES IMPORTED_LOCATION "${BHUMAN_PREFIX}/Util/flite/lib/${PLATFORM}/libflite.a")
endif()

if(NOT ${PLATFORM} STREQUAL macOSarm64)
  include("../CMake/asmjit.cmake")
  include("../CMake/CompiledNN.cmake")
endif()

if(BUILD_DESKTOP)
  set(SIMROBOT_CONTROLLERS SimulatedNao)
  if(APPLE)
    set(SIMROBOT_LIBRARY_DIR "${OUTPUT_PREFIX}/Build/${OS}/SimRobot/$<CONFIG>/SimRobot.app/Contents/lib")
    if(${PLATFORM} STREQUAL macOSarm64)
      set(CONTROLLER_FRAMEWORKS "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.framework")
    endif()
  else()
    set(SIMROBOT_LIBRARY_DIR "${OUTPUT_PREFIX}/Build/${OS}/SimRobot/$<CONFIG>")
  endif()

  include("${SIMROBOT_PREFIX}/Make/Common/SimRobot.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/SimRobotCommon.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/SimRobotCore2.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/SimRobotCore2D.cmake")
  include("${SIMROBOT_PREFIX}/Make/Common/SimRobotEditor.cmake")

  include("../CMake/qtpropertybrowser.cmake")

  if(${CMAKE_BUILD_TYPE} STREQUAL Develop)
    file(GLOB MOVE_TO_CONTROLLER_SOURCES
        "${BHUMAN_PREFIX}/Src/Tools/ImageProcessing/*.cpp" "${BHUMAN_PREFIX}/Src/Tools/ImageProcessing/*.h"
        "${BHUMAN_PREFIX}/Src/Tools/ImageProcessing/CNS/*.cpp" "${BHUMAN_PREFIX}/Src/Tools/ImageProcessing/CNS/*.h"
        "${BHUMAN_PREFIX}/Src/Tools/Math/*.cpp" "${BHUMAN_PREFIX}/Src/Tools/Math/*.h"
        "${BHUMAN_PREFIX}/Src/Tools/Motion/*.cpp" "${BHUMAN_PREFIX}/Src/Tools/Motion/*.h")
    list(APPEND MOVE_TO_CONTROLLER_SOURCES
        "${BHUMAN_PREFIX}/Src/Modules/BehaviorControl/FieldRatingProvider/FieldRatingProvider.h" "${BHUMAN_PREFIX}/Src/Modules/BehaviorControl/FieldRatingProvider/FieldRatingProvider.cpp"
        "${BHUMAN_PREFIX}/Src/Representations/Sensing/RobotModel.cpp" "${BHUMAN_PREFIX}/Src/Representations/Sensing/RobotModel.h")
    if(NOT ${PLATFORM} STREQUAL Windows)
      list(APPEND MOVE_TO_CONTROLLER_SOURCES
          "${BHUMAN_PREFIX}/Src/Modules/MotionControl/WalkingEngine/WalkingEngine.cpp" "${BHUMAN_PREFIX}/Src/Modules/MotionControl/WalkingEngine/WalkingEngine.h"
          "${BHUMAN_PREFIX}/Src/Modules/Perception/ImagePreprocessors/ECImageProvider.cpp" "${BHUMAN_PREFIX}/Src/Modules/Perception/ImagePreprocessors/ECImageProvider.h"
          "${BHUMAN_PREFIX}/Src/Modules/Sensing/FallDownStateDetector/FallDownStateProvider.cpp" "${BHUMAN_PREFIX}/Src/Modules/Sensing/FallDownStateDetector/FallDownStateProvider.h"
          "${BHUMAN_PREFIX}/Src/Modules/Sensing/InertialDataProvider/InertialDataProvider.cpp" "${BHUMAN_PREFIX}/Src/Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
          "${BHUMAN_PREFIX}/Src/Tools/Modeling/UKFPose2D.cpp" "${BHUMAN_PREFIX}/Src/Tools/Modeling/UKFPose2D.h")
    endif()
  endif()

  include("../CMake/Controller.cmake")
  include("../CMake/SimulatedNao.cmake")

  include("../CMake/bush.cmake")
  include("../CMake/Tests.cmake")

  if(APPLE)
    set_property(TARGET SimRobot PROPERTY FOLDER Utils)
    set_property(TARGET SimRobotCommon PROPERTY FOLDER Utils/Libs)
    set_property(TARGET SimRobotCore2 PROPERTY FOLDER Utils/Libs)
    set_property(TARGET SimRobotCore2D PROPERTY FOLDER Utils/Libs)
    set_property(TARGET SimRobotEditor PROPERTY FOLDER Utils/Libs)
    set_property(TARGET SimRobot SimRobotCore2 SimRobotCore2D SimRobotEditor ${SIMROBOT_CONTROLLERS} PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "B-Human Code Release")
  endif()
endif()

if(BUILD_NAO OR (NOT DISABLE_NAO))
  if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "x86_64")
    include("../CMake/asmjitNao.cmake")

    target_compile_definitions(asmjitNao PUBLIC LINUX)
    target_compile_options(asmjitNao PUBLIC
        -UMACOS
        -Qunused-arguments
        -nostdinc
        -nostdinc++
        -march=silvermont
        -mtune=silvermont
        -mrdrnd
        -target x86_64-linux-gnu)
    target_compile_options(asmjitNao PRIVATE
        -Wno-macro-redefined
        -Wno-implicit-int-conversion
        -Wno-enum-enum-conversion)
    target_include_directories(asmjitNao SYSTEM PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/CompiledNN/3rdParty/asmjit/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/CompiledNN/Src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/gcc/include/c++/9"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/gcc/include/x86_64-linux-gnu/c++/9"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/gcc/include/c++/9/backward"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/gcc/include/x86_64-linux-gnu"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/gcc/include")
    target_link_directories(asmjitNao PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/V6/gcc/lib")

    include("../CMake/CompiledNNNao.cmake")
    set_property(TARGET asmjitNao CompiledNNNao PROPERTY XCODE_ATTRIBUTE_LIBTOOL "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/libtool")
  endif()

  include("../CMake/Nao.cmake")
  set(CMAKE_OSX_ARCHITECTURES "")
endif()

if(BUILD_DESKTOP AND (NOT DISABLE_NAO))
  include("../CMake/deploy.cmake")
endif()

source_group(PCH REGULAR_EXPRESSION ".*[ch]xx$")
source_group(Visualizers REGULAR_EXPRESSION ".*natvis$")
