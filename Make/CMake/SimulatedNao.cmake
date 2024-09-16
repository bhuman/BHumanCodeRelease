set(SIMULATEDNAO_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/SimulatedNao")

file(GLOB SIMULATEDNAO_SOURCES CONFIGURE_DEPENDS
    "${SIMULATEDNAO_ROOT_DIR}/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Platform/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Platform/*.h")
file(GLOB_RECURSE SIMULATEDNAO_SOURCES_ADDITIONAL CONFIGURE_DEPENDS
    "${SIMULATEDNAO_ROOT_DIR}/Platform/${PLATFORM}/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Platform/${PLATFORM}/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Representations/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Representations/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Views/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Views/*.h"
    "${SIMULATEDNAO_ROOT_DIR}/Visualization/*.cpp" "${SIMULATEDNAO_ROOT_DIR}/Visualization/*.h")
list(APPEND SIMULATEDNAO_SOURCES ${SIMULATEDNAO_SOURCES_ADDITIONAL})
list(APPEND SIMULATEDNAO_SOURCES "${SIMULATEDNAO_ROOT_DIR}/Controller.qrc")

add_library(SimulatedNao MODULE ${SIMULATEDNAO_SOURCES})

if(MACOS)
  target_include_directories(SimulatedNao SYSTEM PRIVATE ${IO_KIT_FRAMEWORK} ${IO_KIT_FRAMEWORK}/Headers)
  target_link_libraries(SimulatedNao PRIVATE ${IO_KIT_FRAMEWORK})
  target_link_options(SimulatedNao PRIVATE $<$<CONFIG:Develop>:-Xlinker -no_deduplicate>)
endif()

set_property(TARGET SimulatedNao PROPERTY FOLDER Libs)
set_property(TARGET SimulatedNao PROPERTY LIBRARY_OUTPUT_DIRECTORY "${SIMROBOT_LIBRARY_DIR}")
set_property(TARGET SimulatedNao PROPERTY PDB_OUTPUT_DIRECTORY "${SIMROBOT_LIBRARY_DIR}")
set_property(TARGET SimulatedNao PROPERTY AUTOMOC ON)
set_property(TARGET SimulatedNao PROPERTY AUTORCC ON)
target_include_directories(SimulatedNao PRIVATE "${SIMULATEDNAO_ROOT_DIR}/..")
target_link_libraries(SimulatedNao PRIVATE $<$<PLATFORM_ID:Windows>:winmm>)
target_link_libraries(SimulatedNao PRIVATE Qt6::Core Qt6::Gui Qt6::OpenGLWidgets Qt6::Svg Qt6::SvgWidgets Qt6::Widgets)
if(MACOS)
  target_link_libraries(SimulatedNao PRIVATE AppleHelper)
endif()
target_link_libraries(SimulatedNao PRIVATE B-Human)
target_link_libraries(SimulatedNao PRIVATE qtpropertybrowser)
target_link_libraries(SimulatedNao PRIVATE SimRobotInterface)
target_link_libraries(SimulatedNao PRIVATE SimRobotCore2Interface)
target_link_libraries(SimulatedNao PRIVATE SimRobotCore2DInterface)
target_link_libraries(SimulatedNao PRIVATE SimRobotEditorInterface)
if(NOT MACOS)
  target_link_libraries(SimulatedNao PRIVATE asmjit)
endif()
target_link_libraries(SimulatedNao PRIVATE snappy::snappy)
target_link_libraries(SimulatedNao PRIVATE Flags::Default)
target_link_options(SimulatedNao PRIVATE $<${_is_msvc}:$<$<CONFIG:Develop>:/DEBUG>>)

source_group(TREE "${SIMULATEDNAO_ROOT_DIR}" FILES ${SIMULATEDNAO_SOURCES})

if(WINDOWS)
  add_custom_command(TARGET SimRobot POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:FFTW::FFTW>" "$<TARGET_FILE:FFTW::FFTWF>" "$<TARGET_FILE_DIR:SimRobot>")
endif()
