set(ROBOTPARTS_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/RobotParts")

set(ROBOTPARTS_SOURCES
    "${ROBOTPARTS_ROOT_DIR}/Arms.h"
    "${ROBOTPARTS_ROOT_DIR}/FootShape.cpp"
    "${ROBOTPARTS_ROOT_DIR}/FootShape.h"
    "${ROBOTPARTS_ROOT_DIR}/FsrSensors.h"
    "${ROBOTPARTS_ROOT_DIR}/Joints.h"
    "${ROBOTPARTS_ROOT_DIR}/Legs.h"
    "${ROBOTPARTS_ROOT_DIR}/Limbs.h")

add_library(RobotParts${TARGET_SUFFIX} OBJECT ${ROBOTPARTS_SOURCES})
target_sources(RobotParts${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:Math${TARGET_SUFFIX}> $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}>)
set_property(TARGET RobotParts${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET RobotParts${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(RobotParts${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_options(RobotParts${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(RobotParts${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_link_libraries(RobotParts${TARGET_SUFFIX} PRIVATE Math${TARGET_SUFFIX})
target_link_libraries(RobotParts${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_include_directories(RobotParts${TARGET_SUFFIX} PUBLIC "${ROBOTPARTS_ROOT_DIR}/..")
source_group(TREE "${ROBOTPARTS_ROOT_DIR}" FILES ${ROBOTPARTS_SOURCES})
