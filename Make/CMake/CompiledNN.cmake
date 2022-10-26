set(COMPILEDNN_ROOT_DIR "${BHUMAN_PREFIX}/Util/CompiledNN/Src")

file(GLOB_RECURSE COMPILEDNN_SOURCES CONFIGURE_DEPENDS
    "${COMPILEDNN_ROOT_DIR}/*.cpp" "${COMPILEDNN_ROOT_DIR}/*.h")
list(REMOVE_ITEM COMPILEDNN_SOURCES "${COMPILEDNN_ROOT_DIR}/CompiledNN/Formats/ONNX.cpp")

add_library(CompiledNN${TARGET_SUFFIX} STATIC ${COMPILEDNN_SOURCES})
set_property(TARGET CompiledNN${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET CompiledNN${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(CompiledNN${TARGET_SUFFIX} PRIVATE HDF5::HDF5)
else()
  target_link_libraries(CompiledNN${TARGET_SUFFIX} PRIVATE Nao::HDF5::HDF5)
endif()
target_include_directories(CompiledNN${TARGET_SUFFIX} SYSTEM PUBLIC "${COMPILEDNN_ROOT_DIR}")
target_link_libraries(CompiledNN${TARGET_SUFFIX} PUBLIC asmjit${TARGET_SUFFIX})
target_link_libraries(CompiledNN${TARGET_SUFFIX} PUBLIC MathBase${TARGET_SUFFIX})
target_link_libraries(CompiledNN${TARGET_SUFFIX} PUBLIC Platform${TARGET_SUFFIX})
target_link_libraries(CompiledNN${TARGET_SUFFIX} PRIVATE Streaming${TARGET_SUFFIX})
target_compile_definitions(CompiledNN${TARGET_SUFFIX} PRIVATE WITH_KERAS_HDF5)
target_precompile_headers(CompiledNN${TARGET_SUFFIX} PRIVATE "${COMPILEDNN_ROOT_DIR}/CompiledNN/CompiledNN/CompiledNNImplBase.h")

target_link_libraries(CompiledNN${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)

source_group(TREE "${COMPILEDNN_ROOT_DIR}/CompiledNN" FILES ${COMPILEDNN_SOURCES})
