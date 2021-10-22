set(CONTROLLER_ROOT_DIR "${BHUMAN_PREFIX}/Src")
set(CONTROLLER_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/Controller/$<CONFIG>")

file(GLOB CONTROLLER_SOURCES
    "${CONTROLLER_ROOT_DIR}/Controller/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/*.h"
    "${CONTROLLER_ROOT_DIR}/Controller/Platform/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/Platform/*.h")
file(GLOB_RECURSE CONTROLLER_SOURCES_ADDITIONAL
    "${CONTROLLER_ROOT_DIR}/Controller/Platform/${OS}/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/Platform/${OS}/*.h"
    "${CONTROLLER_ROOT_DIR}/Controller/Representations/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/Representations/*.h"
    "${CONTROLLER_ROOT_DIR}/Controller/Trainer/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/Trainer/*.h"
    "${CONTROLLER_ROOT_DIR}/Controller/Views/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/Views/*.h"
    "${CONTROLLER_ROOT_DIR}/Controller/Visualization/*.cpp" "${CONTROLLER_ROOT_DIR}/Controller/Visualization/*.h")
if(APPLE)
  list(APPEND CONTROLLER_SOURCES_ADDITIONAL "${CONTROLLER_ROOT_DIR}/Controller/Visualization/Helper.mm")
endif()
list(APPEND CONTROLLER_SOURCES ${CONTROLLER_SOURCES_ADDITIONAL})
if(${CMAKE_BUILD_TYPE} STREQUAL Develop)
  list(APPEND CONTROLLER_SOURCES ${MOVE_TO_CONTROLLER_SOURCES})
endif()
list(APPEND CONTROLLER_SOURCES "${CONTROLLER_ROOT_DIR}/Controller/Controller.qrc")

if(${PLATFORM} STREQUAL Windows)
  add_library(Controller OBJECT EXCLUDE_FROM_ALL ${CONTROLLER_SOURCES} "${BHUMAN_PREFIX}/Util/Buildchain/Windows/Visualizers/Angle.natvis")
else()
  add_library(Controller OBJECT EXCLUDE_FROM_ALL ${CONTROLLER_SOURCES})
endif()
set_property(TARGET Controller PROPERTY FOLDER Libs)
set_property(TARGET Controller PROPERTY ARCHIVE_OUTPUT_DIRECTORY "${CONTROLLER_OUTPUT_DIR}")
set_property(TARGET Controller PROPERTY AUTOMOC ON)
set_property(TARGET Controller PROPERTY AUTORCC ON)
set_property(TARGET Controller PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(Controller PRIVATE "${CONTROLLER_ROOT_DIR}")
target_include_directories(Controller PRIVATE $<$<PLATFORM_ID:Windows>:${BHUMAN_PREFIX}/Util/Buildchain/Windows/include>)
if(${PLATFORM} STREQUAL macOSarm64)
  target_include_directories(Controller SYSTEM PUBLIC "${BHUMAN_PREFIX}/Util/sse2neon/include")
endif()
target_link_libraries(Controller PRIVATE GameController::GameController)
target_link_libraries(Controller PRIVATE Eigen::Eigen)
target_link_libraries(Controller PRIVATE snappy::snappy)
target_link_libraries(Controller PRIVATE libjpeg::libjpeg)
target_link_libraries(Controller PUBLIC OpenGL::GL)
if(NOT APPLE)
  target_link_libraries(Controller PUBLIC GLEW::GLEW OpenGL::GLU)
endif()
target_link_libraries(Controller PUBLIC Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Svg Qt5::Widgets)
target_link_libraries(Controller PUBLIC $<$<PLATFORM_ID:Windows>:winmm> $<$<PLATFORM_ID:Windows>:ws2_32>)
target_link_libraries(Controller PUBLIC qtpropertybrowser)
target_link_libraries(Controller PRIVATE SimRobotInterface)
target_link_libraries(Controller PUBLIC SimRobotCore2Interface)
target_link_libraries(Controller PUBLIC SimRobotCore2DInterface)
target_link_libraries(Controller PRIVATE SimRobotEditorInterface)
if(${PLATFORM} STREQUAL macOSarm64)
  target_include_directories(Controller SYSTEM PRIVATE "${BHUMAN_PREFIX}/Util/onnxruntime/lib/${PLATFORM}/onnxruntime.framework/Headers")
else()
  target_link_libraries(Controller PRIVATE asmjit)
endif()

target_compile_definitions(Controller PRIVATE TARGET_SIM)

if(MSVC)
  target_compile_options(Controller PRIVATE /Zm200)
else()
  target_compile_options(Controller PRIVATE -Wno-return-stack-address -Wno-switch)
endif()

target_link_libraries(Controller PRIVATE Flags::Default)
target_precompile_headers(Controller PRIVATE "${CONTROLLER_ROOT_DIR}/Tools/Precompiled/BHumanPch.h")

source_group(TREE "${CONTROLLER_ROOT_DIR}" FILES ${CONTROLLER_SOURCES})
