set(COMPILEDNN_ROOT_DIR "${BHUMAN_PREFIX}/Util/CompiledNN/Src")

file(GLOB_RECURSE COMPILEDNN_SOURCES "${COMPILEDNN_ROOT_DIR}/*.cpp" "${COMPILEDNN_ROOT_DIR}/*.h")
list(REMOVE_ITEM COMPILEDNN_SOURCES "${COMPILEDNN_ROOT_DIR}/CompiledNN/Formats/ONNX.cpp")

add_library(CompiledNN STATIC EXCLUDE_FROM_ALL ${COMPILEDNN_SOURCES})
set_property(TARGET CompiledNN PROPERTY ARCHIVE_OUTPUT_DIRECTORY "${COMPILEDNN_OUTPUT_DIR}")
set_property(TARGET CompiledNN PROPERTY FOLDER Libs)
if(BUILD_DESKTOP)
  set_property(TARGET CompiledNN PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(CompiledNN PUBLIC HDF5::HDF5)
else()
  target_link_libraries(CompiledNN PUBLIC Nao::HDF5::HDF5)
endif()
target_include_directories(CompiledNN SYSTEM PUBLIC "${COMPILEDNN_ROOT_DIR}")
target_link_libraries(CompiledNN PUBLIC asmjit)
target_include_directories(CompiledNN PRIVATE "${BHUMAN_PREFIX}/Src")
target_compile_definitions(CompiledNN PRIVATE WITH_KERAS_HDF5)

target_link_libraries(CompiledNN PRIVATE Flags::ForDevelop)

source_group(TREE "${COMPILEDNN_ROOT_DIR}/CompiledNN" FILES ${COMPILEDNN_SOURCES})
