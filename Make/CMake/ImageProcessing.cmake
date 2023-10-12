set(IMAGEPROCESSING_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/ImageProcessing")

set(IMAGEPROCESSING_SOURCES
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CameraModelOpenCV.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CameraModelOpenCV.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CNSResponse.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CNSSSE.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CNSSSE.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CodedContour.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CodedContour.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CylinderRing.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/CylinderRing.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/IsometryWithResponse.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/LutRasterizer.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/LutRasterizer.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/ObjectCNSStereoDetector.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/ObjectCNSStereoDetector.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/ResponseMapping.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/SearchSpecification.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/SubpixelMaximizer.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/SubpixelMaximizer.h"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/TriangleMesh.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/CNS/TriangleMesh.h"
    "${IMAGEPROCESSING_ROOT_DIR}/AVX.h"
    "${IMAGEPROCESSING_ROOT_DIR}/ColorModelConversions.h"
    "${IMAGEPROCESSING_ROOT_DIR}/Image.h"
    "${IMAGEPROCESSING_ROOT_DIR}/ImageTransform.h"
    "${IMAGEPROCESSING_ROOT_DIR}/LabelImage.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/LabelImage.h"
    "${IMAGEPROCESSING_ROOT_DIR}/PatchUtilities.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/PatchUtilities.h"
    "${IMAGEPROCESSING_ROOT_DIR}/PixelTypes.h"
    "${IMAGEPROCESSING_ROOT_DIR}/Resize.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/Resize.h"
    "${IMAGEPROCESSING_ROOT_DIR}/SIMD.h"
    "${IMAGEPROCESSING_ROOT_DIR}/Sobel.cpp"
    "${IMAGEPROCESSING_ROOT_DIR}/Sobel.h"
    "${IMAGEPROCESSING_ROOT_DIR}/YHSColorConversion.h")

add_library(ImageProcessing${TARGET_SUFFIX} OBJECT ${IMAGEPROCESSING_SOURCES})
target_sources(ImageProcessing${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:Math${TARGET_SUFFIX}> $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}> $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}>)
set_property(TARGET ImageProcessing${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET ImageProcessing${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  if(ARM)
    target_include_directories(ImageProcessing${TARGET_SUFFIX} SYSTEM PUBLIC "${BHUMAN_PREFIX}/Util/sse2neon/include")
  endif()
else()
  target_compile_definitions(ImageProcessing${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(ImageProcessing${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
endif()
target_link_libraries(ImageProcessing${TARGET_SUFFIX} PUBLIC Eigen::Eigen)  # for CNS
target_link_libraries(ImageProcessing${TARGET_SUFFIX} PUBLIC Math${TARGET_SUFFIX})
target_link_libraries(ImageProcessing${TARGET_SUFFIX} PUBLIC Platform${TARGET_SUFFIX})
target_link_libraries(ImageProcessing${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_link_libraries(ImageProcessing${TARGET_SUFFIX} PRIVATE Flags::Default)
target_include_directories(ImageProcessing${TARGET_SUFFIX} PUBLIC "${IMAGEPROCESSING_ROOT_DIR}/..")
source_group(TREE "${IMAGEPROCESSING_ROOT_DIR}" FILES ${IMAGEPROCESSING_SOURCES})
