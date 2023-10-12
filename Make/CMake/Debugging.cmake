set(DEBUGGING_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Debugging")

set(DEBUGGING_SOURCES
    "${DEBUGGING_ROOT_DIR}/Annotation.h"
    "${DEBUGGING_ROOT_DIR}/AnnotationManager.cpp"
    "${DEBUGGING_ROOT_DIR}/AnnotationManager.h"
    "${DEBUGGING_ROOT_DIR}/ColorRGBA.cpp"
    "${DEBUGGING_ROOT_DIR}/ColorRGBA.h"
    "${DEBUGGING_ROOT_DIR}/DebugDataStreamer.cpp"
    "${DEBUGGING_ROOT_DIR}/DebugDataStreamer.h"
    "${DEBUGGING_ROOT_DIR}/DebugDataTable.cpp"
    "${DEBUGGING_ROOT_DIR}/DebugDataTable.h"
    "${DEBUGGING_ROOT_DIR}/DebugDrawings.cpp"
    "${DEBUGGING_ROOT_DIR}/DebugDrawings.h"
    "${DEBUGGING_ROOT_DIR}/DebugDrawings3D.h"
    "${DEBUGGING_ROOT_DIR}/Debugging.h"
    "${DEBUGGING_ROOT_DIR}/DebugImages.h"
    "${DEBUGGING_ROOT_DIR}/DebugRequest.cpp"
    "${DEBUGGING_ROOT_DIR}/DebugRequest.h"
    "${DEBUGGING_ROOT_DIR}/Modify.h"
    "${DEBUGGING_ROOT_DIR}/Plot.h"
    "${DEBUGGING_ROOT_DIR}/Stopwatch.h"
    "${DEBUGGING_ROOT_DIR}/TcpConnection.cpp"
    "${DEBUGGING_ROOT_DIR}/TcpConnection.h"
    "${DEBUGGING_ROOT_DIR}/TimingManager.cpp"
    "${DEBUGGING_ROOT_DIR}/TimingManager.h")

add_library(Debugging${TARGET_SUFFIX} OBJECT ${DEBUGGING_SOURCES})
target_sources(Debugging${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:ImageProcessing${TARGET_SUFFIX}> $<TARGET_OBJECTS:Math${TARGET_SUFFIX}> $<TARGET_OBJECTS:Network${TARGET_SUFFIX}> $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}> $<TARGET_OBJECTS:RobotParts${TARGET_SUFFIX}> $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}>)
set_property(TARGET Debugging${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET Debugging${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(Debugging${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(Debugging${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(Debugging${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(Debugging${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_link_libraries(Debugging${TARGET_SUFFIX} PUBLIC ImageProcessing${TARGET_SUFFIX})
target_link_libraries(Debugging${TARGET_SUFFIX} PRIVATE Math${TARGET_SUFFIX})
target_link_libraries(Debugging${TARGET_SUFFIX} PUBLIC Network${TARGET_SUFFIX})
target_link_libraries(Debugging${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
target_link_libraries(Debugging${TARGET_SUFFIX} PUBLIC RobotParts${TARGET_SUFFIX})
target_link_libraries(Debugging${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_include_directories(Debugging${TARGET_SUFFIX} PUBLIC "${DEBUGGING_ROOT_DIR}/..")
source_group(TREE "${DEBUGGING_ROOT_DIR}" FILES ${DEBUGGING_SOURCES})
