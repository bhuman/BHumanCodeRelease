set(MATHBASE_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/MathBase")

set(MATHBASE_SOURCES
    "${MATHBASE_ROOT_DIR}/Angle.h"
    "${MATHBASE_ROOT_DIR}/Approx.h"
    "${MATHBASE_ROOT_DIR}/BHMath.h"
    "${MATHBASE_ROOT_DIR}/Constants.h"
    "${MATHBASE_ROOT_DIR}/Covariance.cpp"
    "${MATHBASE_ROOT_DIR}/Covariance.h"
    "${MATHBASE_ROOT_DIR}/Deviation.h"
    "${MATHBASE_ROOT_DIR}/Eigen.h"
    "${MATHBASE_ROOT_DIR}/EigenArrayExtensions.h"
    "${MATHBASE_ROOT_DIR}/EigenMatrixBaseExtensions.h"
    "${MATHBASE_ROOT_DIR}/GaussNewtonOptimizer.h"
    "${MATHBASE_ROOT_DIR}/LeastSquares.cpp"
    "${MATHBASE_ROOT_DIR}/LeastSquares.h"
    "${MATHBASE_ROOT_DIR}/MeanCalculator.h"
    "${MATHBASE_ROOT_DIR}/NeumaierSum.h"
    "${MATHBASE_ROOT_DIR}/Probabilistics.h"
    "${MATHBASE_ROOT_DIR}/Random.cpp"
    "${MATHBASE_ROOT_DIR}/Random.h"
    "${MATHBASE_ROOT_DIR}/RingBuffer.h"
    "${MATHBASE_ROOT_DIR}/RingBufferWithSum.h"
    "${MATHBASE_ROOT_DIR}/Rotation.h"
    "${MATHBASE_ROOT_DIR}/UnscentedKalmanFilter.h")

add_library(MathBase${TARGET_SUFFIX} OBJECT ${MATHBASE_SOURCES})
target_sources(MathBase${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}>)
if(WINDOWS AND NOT MINIMAL_PROJECT)
  target_sources(MathBase${TARGET_SUFFIX} INTERFACE "${BHUMAN_PREFIX}/Util/Buildchain/Windows/Visualizers/Angle.natvis")
endif()
set_property(TARGET MathBase${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET MathBase${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
else()
  target_compile_definitions(MathBase${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(MathBase${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
endif()
target_link_libraries(MathBase${TARGET_SUFFIX} PUBLIC Eigen::Eigen)
target_link_libraries(MathBase${TARGET_SUFFIX} PUBLIC Platform${TARGET_SUFFIX})
target_link_libraries(MathBase${TARGET_SUFFIX} PRIVATE Flags::Default)
target_precompile_headers(MathBase${TARGET_SUFFIX} PUBLIC
    "${MATHBASE_ROOT_DIR}/BHMath.h"
    "${MATHBASE_ROOT_DIR}/Eigen.h"
    "${MATHBASE_ROOT_DIR}/RingBufferWithSum.h")
target_include_directories(MathBase${TARGET_SUFFIX} PUBLIC "${MATHBASE_ROOT_DIR}/..")
source_group(TREE "${MATHBASE_ROOT_DIR}" FILES ${MATHBASE_SOURCES})
