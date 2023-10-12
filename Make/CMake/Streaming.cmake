set(STREAMING_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Streaming")

set(STREAMING_SOURCES
    "${STREAMING_ROOT_DIR}/AlignedMemory.cpp"
    "${STREAMING_ROOT_DIR}/AlignedMemory.h"
    "${STREAMING_ROOT_DIR}/AutoStreamable.h"
    "${STREAMING_ROOT_DIR}/Enum.h"
    "${STREAMING_ROOT_DIR}/EnumIndexedArray.h"
    "${STREAMING_ROOT_DIR}/Function.h"
    "${STREAMING_ROOT_DIR}/FunctionList.cpp"
    "${STREAMING_ROOT_DIR}/FunctionList.h"
    "${STREAMING_ROOT_DIR}/Global.cpp"
    "${STREAMING_ROOT_DIR}/Global.h"
    "${STREAMING_ROOT_DIR}/InExpr.cpp"
    "${STREAMING_ROOT_DIR}/InExpr.h"
    "${STREAMING_ROOT_DIR}/InOut.cpp"
    "${STREAMING_ROOT_DIR}/InOut.h"
    "${STREAMING_ROOT_DIR}/InStreams.cpp"
    "${STREAMING_ROOT_DIR}/InStreams.h"
    "${STREAMING_ROOT_DIR}/MessageIDs.h"
    "${STREAMING_ROOT_DIR}/MessageQueue.cpp"
    "${STREAMING_ROOT_DIR}/MessageQueue.h"
    "${STREAMING_ROOT_DIR}/Output.cpp"
    "${STREAMING_ROOT_DIR}/Output.h"
    "${STREAMING_ROOT_DIR}/OutStreams.cpp"
    "${STREAMING_ROOT_DIR}/OutStreams.h"
    "${STREAMING_ROOT_DIR}/SimpleMap.cpp"
    "${STREAMING_ROOT_DIR}/SimpleMap.h"
    "${STREAMING_ROOT_DIR}/Streamable.cpp"
    "${STREAMING_ROOT_DIR}/Streamable.h"
    "${STREAMING_ROOT_DIR}/TypeInfo.cpp"
    "${STREAMING_ROOT_DIR}/TypeInfo.h"
    "${STREAMING_ROOT_DIR}/TypeRegistry.cpp"
    "${STREAMING_ROOT_DIR}/TypeRegistry.h")

add_library(Streaming${TARGET_SUFFIX} OBJECT ${STREAMING_SOURCES})
target_sources(Streaming${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:MathBase${TARGET_SUFFIX}> $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}>)
set_property(TARGET Streaming${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET Streaming${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(Streaming${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(Streaming${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(Streaming${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(Streaming${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_compile_options(Streaming${TARGET_SUFFIX} INTERFACE $<${_is_clang}:-Wno-switch>)  # for ENUMs
target_link_libraries(Streaming${TARGET_SUFFIX} PRIVATE MathBase${TARGET_SUFFIX})
target_link_libraries(Streaming${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
target_include_directories(Streaming${TARGET_SUFFIX} PUBLIC "${STREAMING_ROOT_DIR}/..")
source_group(TREE "${STREAMING_ROOT_DIR}" FILES ${STREAMING_SOURCES})
