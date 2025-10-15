set(LOG_PLAYBACK_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/LogPlayback")

file(GLOB_RECURSE LOG_PLAYBACK_SOURCES CONFIGURE_DEPENDS
    "${LOG_PLAYBACK_ROOT_DIR}/*.cpp" "${LOG_PLAYBACK_ROOT_DIR}/*.h")

add_library(LogPlayback OBJECT ${LOG_PLAYBACK_SOURCES})
target_sources(LogPlayback INTERFACE
    $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}>
    $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}>
    $<TARGET_OBJECTS:Debugging${TARGET_SUFFIX}>)
set_property(TARGET LogPlayback PROPERTY FOLDER Libs)
set_property(TARGET LogPlayback PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(LogPlayback PUBLIC "${LOG_PLAYBACK_ROOT_DIR}/..")
target_link_libraries(LogPlayback PRIVATE Debugging)
target_link_libraries(LogPlayback PRIVATE Framework)
target_link_libraries(LogPlayback PRIVATE Platform)
target_link_libraries(LogPlayback PRIVATE Streaming)
target_link_libraries(LogPlayback PRIVATE snappy::snappy)
target_link_libraries(LogPlayback PRIVATE Flags::Default)
target_link_options(LogPlayback PRIVATE $<${_is_msvc}:$<$<CONFIG:Develop>:/DEBUG>>)

source_group(TREE "${LOG_PLAYBACK_ROOT_DIR}" FILES ${LOG_PLAYBACK_SOURCES})
