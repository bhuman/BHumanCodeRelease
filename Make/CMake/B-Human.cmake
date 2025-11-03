set(BHUMAN_ROOT_DIR "${BHUMAN_PREFIX}/Src")

file(GLOB_RECURSE BHUMAN_SOURCES CONFIGURE_DEPENDS
    "${BHUMAN_ROOT_DIR}/Modules/*.cpp" "${BHUMAN_ROOT_DIR}/Modules/*.h"
    "${BHUMAN_ROOT_DIR}/Representations/*.cpp" "${BHUMAN_ROOT_DIR}/Representations/*.h"
    "${BHUMAN_ROOT_DIR}/Threads/*.cpp" "${BHUMAN_ROOT_DIR}/Threads/*.h"
    "${BHUMAN_ROOT_DIR}/Tools/*.cpp" "${BHUMAN_ROOT_DIR}/Tools/*.h")

list(REMOVE_ITEM BHUMAN_SOURCES ${OPTIONS_AND_SKILLS_SOURCES})

file(GLOB BHUMAN_OPTIMIZED_SOURCES CONFIGURE_DEPENDS
    "${BHUMAN_ROOT_DIR}/Tools/Math/*.cpp" "${BHUMAN_ROOT_DIR}/Tools/Math/*.h"
    "${BHUMAN_ROOT_DIR}/Tools/Motion/*.cpp" "${BHUMAN_ROOT_DIR}/Tools/Motion/*.h")
list(APPEND BHUMAN_OPTIMIZED_SOURCES
    "${BHUMAN_ROOT_DIR}/Modules/BehaviorControl/FieldRatingProvider/FieldRatingProvider.h" "${BHUMAN_ROOT_DIR}/Modules/BehaviorControl/FieldRatingProvider/FieldRatingProvider.cpp"
    "${BHUMAN_ROOT_DIR}/Representations/Sensing/RobotModel.cpp" "${BHUMAN_ROOT_DIR}/Representations/Sensing/RobotModel.h")
if(NOT WINDOWS)
  list(APPEND BHUMAN_OPTIMIZED_SOURCES
      "${BHUMAN_ROOT_DIR}/Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.cpp" "${BHUMAN_ROOT_DIR}/Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.h"
      "${BHUMAN_ROOT_DIR}/Modules/MotionControl/WalkingEngine/WalkingEngine.cpp" "${BHUMAN_ROOT_DIR}/Modules/MotionControl/WalkingEngine/WalkingEngine.h"
      "${BHUMAN_ROOT_DIR}/Modules/Perception/FieldPerceptors/LinePerceptor.cpp" "${BHUMAN_ROOT_DIR}/Modules/Perception/FieldPerceptors/LinePerceptor.h"
      "${BHUMAN_ROOT_DIR}/Modules/Perception/ImagePreprocessors/ECImageProvider.cpp" "${BHUMAN_ROOT_DIR}/Modules/Perception/ImagePreprocessors/ECImageProvider.h"
      "${BHUMAN_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/FallDownStateProvider.cpp" "${BHUMAN_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/FallDownStateProvider.h"
      "${BHUMAN_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/BoosterFallDownStateProvider.cpp" "${BHUMAN_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/BoosterFallDownStateProvider.h"
      "${BHUMAN_ROOT_DIR}/Modules/Sensing/InertialDataProvider/InertialDataProvider.cpp" "${BHUMAN_ROOT_DIR}/Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
      "${BHUMAN_ROOT_DIR}/Tools/Modeling/UKFPose2D.cpp" "${BHUMAN_ROOT_DIR}/Tools/Modeling/UKFPose2D.h")
endif()

set(BHUMAN_PCHS
    [["Debugging/DebugDrawings.h"]]
    [["Framework/Module.h"]]
    [["MathBase/RingBufferWithSum.h"]]
    [["Math/Eigen.h"]]
    [["Math/Boundary.h"]]
    [["Math/Geometry.h"]]
    [["Math/Pose3f.h"]]
    [["Streaming/MessageQueue.h"]]
    [["Tools/Math/Transformation.h"]]
    <algorithm>
    <list>
    <sstream>)

if(MACOS AND BUILD_DESKTOP)
  add_library(B-Human-Optimized${TARGET_SUFFIX} OBJECT ${BHUMAN_OPTIMIZED_SOURCES})
  set_property(TARGET B-Human-Optimized${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
  set_property(TARGET B-Human-Optimized${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_include_directories(B-Human-Optimized${TARGET_SUFFIX} PRIVATE "${BHUMAN_ROOT_DIR}")
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE MathBase${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Streaming${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Math${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE ImageProcessing${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Network${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE RobotParts${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Debugging${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Framework${TARGET_SUFFIX})
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE CompiledNN::ONNX)
  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PUBLIC asmjit${TARGET_SUFFIX})

  target_link_libraries(B-Human-Optimized${TARGET_SUFFIX} PRIVATE Flags::Default)
  target_precompile_headers(B-Human-Optimized${TARGET_SUFFIX} PRIVATE ${BHUMAN_PCHS})

  list(REMOVE_ITEM BHUMAN_SOURCES ${BHUMAN_OPTIMIZED_SOURCES})
endif()

add_library(B-Human${TARGET_SUFFIX} OBJECT ${BHUMAN_SOURCES})
if(BUILD_DESKTOP)
  if(MACOS)
    target_sources(B-Human${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:B-Human-Optimized${TARGET_SUFFIX}>)
    target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC B-Human-Optimized${TARGET_SUFFIX})
  else()
    set_property(SOURCE ${BHUMAN_OPTIMIZED_SOURCES} APPEND PROPERTY COMPILE_OPTIONS
        $<${_is_msvc}:$<$<CONFIG:Develop>:/Ox>>
        $<${_is_clang}:$<$<CONFIG:Develop>:-O3>>)
    set_property(SOURCE ${BHUMAN_OPTIMIZED_SOURCES} PROPERTY SKIP_PRECOMPILE_HEADERS ON)
  endif()
endif()
target_sources(B-Human${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:OptionsAndSkills${TARGET_SUFFIX}> $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}> $<TARGET_OBJECTS:MathBase${TARGET_SUFFIX}> $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}> $<TARGET_OBJECTS:Math${TARGET_SUFFIX}> $<TARGET_OBJECTS:ImageProcessing${TARGET_SUFFIX}> $<TARGET_OBJECTS:Network${TARGET_SUFFIX}> $<TARGET_OBJECTS:RobotParts${TARGET_SUFFIX}> $<TARGET_OBJECTS:Debugging${TARGET_SUFFIX}> $<TARGET_OBJECTS:Framework${TARGET_SUFFIX}>)

set_property(TARGET B-Human${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
target_include_directories(B-Human${TARGET_SUFFIX} PUBLIC "${BHUMAN_ROOT_DIR}")
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC OptionsAndSkills${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC Platform${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC MathBase${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC Math${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC ImageProcessing${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC Network${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC RobotParts${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC Debugging${TARGET_SUFFIX})
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC Framework${TARGET_SUFFIX})
if(BUILD_DESKTOP)
  set_property(TARGET B-Human${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE FFTW::FFTW FFTW::FFTWF)
  target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE libjpeg::libjpeg)
  if(OPENSSL_FOUND)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE OpenSSL::SSL)
    target_compile_definitions(B-Human${TARGET_SUFFIX} PRIVATE HAVE_OPENSSL)
  endif()
  if(X86)
    if(NOT MACOS)
      target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE asmjit${TARGET_SUFFIX})
    endif()
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE CompiledNN${TARGET_SUFFIX})
  endif()
  target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE CompiledNN::ONNX)
  target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  if(X86)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE CompiledNN${TARGET_SUFFIX})
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Nao::FFTW::FFTW Nao::FFTW::FFTWF)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Nao::libjpeg::libjpeg)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Nao::ALSA::ALSA)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Nao::CompiledNN::ONNX)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Nao::OpenSSL::SSL)
    target_compile_definitions(B-Human${TARGET_SUFFIX} PRIVATE HAVE_OPENSSL)
  else()
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::FFTW::FFTW Booster::FFTW::FFTWF)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::libjpeg::libjpeg)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::CompiledNN::ONNX)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::realsense2::realsense2)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::Booster::SDK)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::FastCDR::FastCDR)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::FastRTPS::FastRTPS)
    target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Booster::FooNathan::Memory)
  endif()
  target_include_directories(B-Human${TARGET_SUFFIX} SYSTEM PRIVATE "${BHUMAN_PREFIX}/Util/MD5")
  target_compile_definitions(B-Human${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(B-Human${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG> $<$<CONFIG:Release>:-Wno-unused>)
  target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_link_libraries(B-Human${TARGET_SUFFIX} PRIVATE voronoi::voronoi)
target_link_libraries(B-Human${TARGET_SUFFIX} PUBLIC GameController::GameController)
target_compile_definitions(B-Human${TARGET_SUFFIX} PRIVATE CONFIGURATION=$<CONFIG>) # for RobotHealthProvider

target_precompile_headers(B-Human${TARGET_SUFFIX} PUBLIC ${BHUMAN_PCHS})
if(MSVC)
  target_compile_options(B-Human${TARGET_SUFFIX} PUBLIC /Zm200)
endif()

source_group(TREE "${BHUMAN_ROOT_DIR}" FILES ${BHUMAN_SOURCES})
