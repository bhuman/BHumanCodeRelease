set(SIMULATEDNAO_ROOT_DIR "${BHUMAN_PREFIX}/Src")

file(GLOB_RECURSE SIMULATEDNAO_SOURCES
    "${SIMULATEDNAO_ROOT_DIR}/Modules/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Modules/*.h")
file(GLOB SIMULATEDNAO_SOURCES_ADDITIONAL
    "${SIMULATEDNAO_ROOT_DIR}/Platform/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Platform/*.h")
list(APPEND SIMULATEDNAO_SOURCES ${SIMULATEDNAO_SOURCES_ADDITIONAL})
file(GLOB_RECURSE SIMULATEDNAO_SOURCES_ADDITIONAL
    "${SIMULATEDNAO_ROOT_DIR}/Representations/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Representations/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Tools/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Tools/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Platform/${OS}/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Platform/${OS}/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Platform/${OS}/*.mm"
    "${SIMULATEDNAO_ROOT_DIR}/Threads/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Threads/*.h")
list(APPEND SIMULATEDNAO_SOURCES ${SIMULATEDNAO_SOURCES_ADDITIONAL})
if(${CMAKE_BUILD_TYPE} STREQUAL Develop)
  list(REMOVE_ITEM SIMULATEDNAO_SOURCES ${MOVE_TO_CONTROLLER_SOURCES})
endif()

if(${PLATFORM} STREQUAL Windows)
  add_library(SimulatedNao MODULE ${SIMULATEDNAO_SOURCES} $<TARGET_OBJECTS:Controller> "${BHUMAN_PREFIX}/Util/Buildchain/Windows/Visualizers/Angle.natvis")
else()
  add_library(SimulatedNao MODULE ${SIMULATEDNAO_SOURCES} $<TARGET_OBJECTS:Controller>)
endif()

if(APPLE)
  target_include_directories(SimulatedNao SYSTEM PRIVATE ${APP_KIT_FRAMEWORK} ${APP_KIT_FRAMEWORK}/Headers)
  target_link_libraries(SimulatedNao PRIVATE ${APP_KIT_FRAMEWORK})

  target_include_directories(SimulatedNao SYSTEM PRIVATE ${CORE_SERVICES_FRAMEWORK} ${CORE_SERVICES_FRAMEWORK}/Headers)
  target_link_libraries(SimulatedNao PRIVATE ${CORE_SERVICES_FRAMEWORK})

  target_include_directories(SimulatedNao SYSTEM PRIVATE ${IO_KIT_FRAMEWORK} ${IO_KIT_FRAMEWORK}/Headers)
  target_link_libraries(SimulatedNao PRIVATE ${IO_KIT_FRAMEWORK})
endif()

set_property(TARGET SimulatedNao PROPERTY LIBRARY_OUTPUT_DIRECTORY "${SIMROBOT_LIBRARY_DIR}")
set_property(TARGET SimulatedNao PROPERTY PDB_OUTPUT_DIRECTORY "${SIMROBOT_LIBRARY_DIR}")
target_include_directories(SimulatedNao PRIVATE "${SIMULATEDNAO_ROOT_DIR}")
target_include_directories(SimulatedNao PRIVATE $<$<PLATFORM_ID:Windows>:${BHUMAN_PREFIX}/Util/Buildchain/Windows/include>)
target_link_libraries(SimulatedNao PRIVATE Eigen::Eigen)
target_link_libraries(SimulatedNao PRIVATE FFTW::FFTW FFTW::FFTWF)
target_link_libraries(SimulatedNao PRIVATE libjpeg::libjpeg)
target_link_libraries(SimulatedNao PRIVATE snappy::snappy)
target_link_libraries(SimulatedNao PRIVATE $<$<PLATFORM_ID:Linux>:flite::flite_cmu_us_slt> $<$<PLATFORM_ID:Linux>:flite::flite_usenglish>
    $<$<PLATFORM_ID:Linux>:flite::flite_cmulex> $<$<PLATFORM_ID:Linux>:flite::flite>)
target_link_libraries(SimulatedNao PRIVATE $<$<PLATFORM_ID:Linux>:ALSA::ALSA>)
target_link_libraries(SimulatedNao PRIVATE GameController::GameController)
target_link_libraries(SimulatedNao PRIVATE Controller)
if(${PLATFORM} STREQUAL macOSarm64)
  target_link_libraries(SimulatedNao PRIVATE ONNXRuntime::ONNXRuntime)
else()
  target_link_libraries(SimulatedNao PRIVATE asmjit)
  target_link_libraries(SimulatedNao PRIVATE CompiledNN)
endif()
target_compile_definitions(SimulatedNao PRIVATE TARGET_SIM CONFIGURATION=$<CONFIG>)

if(MSVC)
  target_compile_options(SimulatedNao PRIVATE /Zm200 $<$<CONFIG:Release>:/wd4101>)
else()
  target_compile_options(SimulatedNao PRIVATE -Wno-return-stack-address -Wno-switch)
endif()

target_link_libraries(SimulatedNao PRIVATE Flags::ForDevelop)
target_precompile_headers(SimulatedNao PRIVATE "${SIMULATEDNAO_ROOT_DIR}/Tools/Precompiled/BHumanPch.h")

source_group(TREE "${SIMULATEDNAO_ROOT_DIR}" FILES ${SIMULATEDNAO_SOURCES})

if(WIN32)
  add_custom_command(TARGET SimRobot POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:FFTW::FFTW>" "$<TARGET_FILE:FFTW::FFTWF>" "$<TARGET_FILE_DIR:SimRobot>")
endif()
