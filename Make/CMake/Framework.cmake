set(FRAMEWORK_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Framework")

set(FRAMEWORK_SOURCES
    "${FRAMEWORK_ROOT_DIR}/Blackboard.cpp"
    "${FRAMEWORK_ROOT_DIR}/Blackboard.h"
    "${FRAMEWORK_ROOT_DIR}/Configuration.h"
    "${FRAMEWORK_ROOT_DIR}/Debug.cpp"
    "${FRAMEWORK_ROOT_DIR}/Debug.h"
    "${FRAMEWORK_ROOT_DIR}/DebugHandler.cpp"
    "${FRAMEWORK_ROOT_DIR}/DebugHandler.h"
    "${FRAMEWORK_ROOT_DIR}/Communication.cpp"
    "${FRAMEWORK_ROOT_DIR}/Communication.h"
    "${FRAMEWORK_ROOT_DIR}/FrameExecutionUnit.cpp"
    "${FRAMEWORK_ROOT_DIR}/FrameExecutionUnit.h"
    "${FRAMEWORK_ROOT_DIR}/Logger.cpp"
    "${FRAMEWORK_ROOT_DIR}/Logger.h"
    "${FRAMEWORK_ROOT_DIR}/LoggingTools.cpp"
    "${FRAMEWORK_ROOT_DIR}/LoggingTools.h"
    "${FRAMEWORK_ROOT_DIR}/Module.cpp"
    "${FRAMEWORK_ROOT_DIR}/Module.h"
    "${FRAMEWORK_ROOT_DIR}/ModuleContainer.cpp"
    "${FRAMEWORK_ROOT_DIR}/ModuleContainer.h"
    "${FRAMEWORK_ROOT_DIR}/ModuleGraphCreator.cpp"
    "${FRAMEWORK_ROOT_DIR}/ModuleGraphCreator.h"
    "${FRAMEWORK_ROOT_DIR}/ModuleGraphRunner.cpp"
    "${FRAMEWORK_ROOT_DIR}/ModuleGraphRunner.h"
    "${FRAMEWORK_ROOT_DIR}/ModulePacket.h"
    "${FRAMEWORK_ROOT_DIR}/Next.h"
    "${FRAMEWORK_ROOT_DIR}/Robot.cpp"
    "${FRAMEWORK_ROOT_DIR}/Robot.h"
    "${FRAMEWORK_ROOT_DIR}/Settings.cpp"
    "${FRAMEWORK_ROOT_DIR}/Settings.h"
    "${FRAMEWORK_ROOT_DIR}/ThreadFrame.cpp"
    "${FRAMEWORK_ROOT_DIR}/ThreadFrame.h")

add_library(Framework${TARGET_SUFFIX} OBJECT ${FRAMEWORK_SOURCES})
target_sources(Framework${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:Debugging${TARGET_SUFFIX}> $<TARGET_OBJECTS:Network${TARGET_SUFFIX}> $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}> $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}>)
set_property(TARGET Framework${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET Framework${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(Framework${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(Framework${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(Framework${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(Framework${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_link_libraries(Framework${TARGET_SUFFIX} PRIVATE asmjit${TARGET_SUFFIX})
target_link_libraries(Framework${TARGET_SUFFIX} PUBLIC Debugging${TARGET_SUFFIX})
target_link_libraries(Framework${TARGET_SUFFIX} PUBLIC Network${TARGET_SUFFIX})
target_link_libraries(Framework${TARGET_SUFFIX} PUBLIC Platform${TARGET_SUFFIX})
target_link_libraries(Framework${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_include_directories(Framework${TARGET_SUFFIX} PUBLIC "${FRAMEWORK_ROOT_DIR}/..")
source_group(TREE "${FRAMEWORK_ROOT_DIR}" FILES ${FRAMEWORK_SOURCES})
