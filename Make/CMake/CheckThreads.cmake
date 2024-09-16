set(CHECK_THREADS_ROOT_DIR "${BHUMAN_PREFIX}/Src/Apps/CheckThreads")
set(CHECK_THREADS_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/CheckThreads/$<CONFIG>")

file(GLOB_RECURSE CHECK_THREADS_SOURCES CONFIGURE_DEPENDS
    "${CHECK_THREADS_ROOT_DIR}/*.cpp" "${CHECK_THREADS_ROOT_DIR}/*.h")

set(CHECK_THREADS_TREE "${CHECK_THREADS_SOURCES}")

add_executable(CheckThreads EXCLUDE_FROM_ALL ${CHECK_THREADS_SOURCES})

set_property(TARGET CheckThreads PROPERTY RUNTIME_OUTPUT_DIRECTORY "${CHECK_THREADS_OUTPUT_DIR}")
set_property(TARGET CheckThreads PROPERTY FOLDER Apps)
# This is not quite the nice way.
set_property(TARGET CheckThreads PROPERTY XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/../../../../Util/onnxruntime/lib/${PLATFORM}")

target_include_directories(CheckThreads PRIVATE "${CHECK_THREADS_ROOT_DIR}")

target_link_libraries(CheckThreads PRIVATE B-Human)
target_link_libraries(CheckThreads PRIVATE Framework)
target_link_libraries(CheckThreads PRIVATE Streaming)
target_link_libraries(CheckThreads PRIVATE Flags::Default)

source_group(TREE "${CHECK_THREADS_ROOT_DIR}" FILES ${CHECK_THREADS_TREE})
