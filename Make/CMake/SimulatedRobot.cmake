set(SIMULATEDROBOT_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/SimulatedRobot")

file(GLOB SIMULATEDROBOT_SOURCES CONFIGURE_DEPENDS
    "${SIMULATEDROBOT_ROOT_DIR}/*.cpp" "${SIMULATEDROBOT_ROOT_DIR}/*.h"
    "${SIMULATEDROBOT_ROOT_DIR}/Platform/*.cpp" "${SIMULATEDROBOT_ROOT_DIR}/Platform/*.h")
file(GLOB_RECURSE SIMULATEDROBOT_SOURCES_ADDITIONAL CONFIGURE_DEPENDS
    "${SIMULATEDROBOT_ROOT_DIR}/Platform/${PLATFORM}/*.cpp" "${SIMULATEDROBOT_ROOT_DIR}/Platform/${PLATFORM}/*.h"
    "${SIMULATEDROBOT_ROOT_DIR}/Representations/*.cpp" "${SIMULATEDROBOT_ROOT_DIR}/Representations/*.h"
    "${SIMULATEDROBOT_ROOT_DIR}/Views/*.cpp" "${SIMULATEDROBOT_ROOT_DIR}/Views/*.h"
    "${SIMULATEDROBOT_ROOT_DIR}/Visualization/*.cpp" "${SIMULATEDROBOT_ROOT_DIR}/Visualization/*.h")
list(APPEND SIMULATEDROBOT_SOURCES ${SIMULATEDROBOT_SOURCES_ADDITIONAL})
list(APPEND SIMULATEDROBOT_SOURCES "${SIMULATEDROBOT_ROOT_DIR}/Controller.qrc")

add_library(SimulatedRobot MODULE ${SIMULATEDROBOT_SOURCES})

if(MACOS)
  target_include_directories(SimulatedRobot SYSTEM PRIVATE ${IO_KIT_FRAMEWORK} ${IO_KIT_FRAMEWORK}/Headers)
  target_link_libraries(SimulatedRobot PRIVATE ${IO_KIT_FRAMEWORK})
  target_link_options(SimulatedRobot PRIVATE $<$<CONFIG:Develop>:-Xlinker -no_deduplicate>)
endif()

set_property(TARGET SimulatedRobot PROPERTY FOLDER Libs)
set_property(TARGET SimulatedRobot PROPERTY LIBRARY_OUTPUT_DIRECTORY "${SIMROBOT_LIBRARY_DIR}")
set_property(TARGET SimulatedRobot PROPERTY PDB_OUTPUT_DIRECTORY "${SIMROBOT_LIBRARY_DIR}")
set_property(TARGET SimulatedRobot PROPERTY AUTOMOC ON)
set_property(TARGET SimulatedRobot PROPERTY AUTORCC ON)
target_include_directories(SimulatedRobot PRIVATE "${SIMULATEDROBOT_ROOT_DIR}/..")
target_link_libraries(SimulatedRobot PRIVATE $<$<PLATFORM_ID:Windows>:winmm>)
target_link_libraries(SimulatedRobot PRIVATE Qt6::Core Qt6::Gui Qt6::OpenGLWidgets Qt6::Svg Qt6::SvgWidgets Qt6::Widgets)
target_link_libraries(SimulatedRobot PRIVATE B-Human)
target_link_libraries(SimulatedRobot PRIVATE LogPlayback)
target_link_libraries(SimulatedRobot PRIVATE qtpropertybrowser)
target_link_libraries(SimulatedRobot PRIVATE SimRobotInterface)
target_link_libraries(SimulatedRobot PRIVATE SimRobotCore3Interface)
target_link_libraries(SimulatedRobot PRIVATE SimRobotCore2DInterface)
target_link_libraries(SimulatedRobot PRIVATE SimRobotEditorInterface)
if(NOT MACOS)
  target_link_libraries(SimulatedRobot PRIVATE asmjit)
endif()
target_link_libraries(SimulatedRobot PRIVATE Flags::Default)
target_link_options(SimulatedRobot PRIVATE $<${_is_msvc}:$<$<CONFIG:Develop>:/DEBUG>>)

source_group(TREE "${SIMULATEDROBOT_ROOT_DIR}" FILES ${SIMULATEDROBOT_SOURCES})

if(WINDOWS)
  add_custom_command(TARGET SimRobot POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:FFTW::FFTW>" "$<TARGET_FILE:FFTW::FFTWF>" "$<TARGET_FILE_DIR:SimRobot>")
endif()
