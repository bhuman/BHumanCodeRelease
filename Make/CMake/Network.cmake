set(NETWORK_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Network")

set(NETWORK_SOURCES
    "${NETWORK_ROOT_DIR}/RoboCupGameControlData.h"
    "${NETWORK_ROOT_DIR}/TcpComm.cpp"
    "${NETWORK_ROOT_DIR}/TcpComm.h"
    "${NETWORK_ROOT_DIR}/UdpComm.cpp"
    "${NETWORK_ROOT_DIR}/UdpComm.h")

add_library(Network${TARGET_SUFFIX} OBJECT ${NETWORK_SOURCES})
target_sources(Network${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}>)
set_property(TARGET Network${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET Network${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(Network${TARGET_SUFFIX} PUBLIC $<$<PLATFORM_ID:Windows>:ws2_32>)
  target_link_libraries(Network${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(Network${TARGET_SUFFIX} PRIVATE TARGET_ROBOT)
  target_compile_options(Network${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(Network${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_link_libraries(Network${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
target_link_libraries(Network${TARGET_SUFFIX} PUBLIC GameController::GameController)
target_include_directories(Network${TARGET_SUFFIX} PUBLIC "${NETWORK_ROOT_DIR}/..")
source_group(TREE "${NETWORK_ROOT_DIR}" FILES ${NETWORK_SOURCES})
