set(MATH_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Math")

set(MATH_SOURCES
    "${MATH_ROOT_DIR}/Angle.h"
    "${MATH_ROOT_DIR}/Approx.h"
    "${MATH_ROOT_DIR}/BHMath.h"
    "${MATH_ROOT_DIR}/Boundary.h"
    "${MATH_ROOT_DIR}/Constants.h"
    "${MATH_ROOT_DIR}/Covariance.h"
    "${MATH_ROOT_DIR}/Deviation.h"
    "${MATH_ROOT_DIR}/Eigen.h"
    "${MATH_ROOT_DIR}/GaussNewtonOptimizer.h"
    "${MATH_ROOT_DIR}/Geometry.cpp"
    "${MATH_ROOT_DIR}/Geometry.h"
    "${MATH_ROOT_DIR}/LeastSquares.h"
    "${MATH_ROOT_DIR}/MeanCalculator.h"
    "${MATH_ROOT_DIR}/NeumaierSum.h"
    "${MATH_ROOT_DIR}/Pose2f.h"
    "${MATH_ROOT_DIR}/Pose3f.h"
    "${MATH_ROOT_DIR}/Probabilistics.h"
    "${MATH_ROOT_DIR}/Random.h"
    "${MATH_ROOT_DIR}/Range.h"
    "${MATH_ROOT_DIR}/RingBuffer.h"
    "${MATH_ROOT_DIR}/RingBufferWithSum.h"
    "${MATH_ROOT_DIR}/Rotation.h"
    "${MATH_ROOT_DIR}/RotationMatrix.cpp"
    "${MATH_ROOT_DIR}/RotationMatrix.h"
    "${MATH_ROOT_DIR}/SE3fWithCov.cpp"
    "${MATH_ROOT_DIR}/SE3fWithCov.h"
    "${MATH_ROOT_DIR}/SE3Tools.cpp"
    "${MATH_ROOT_DIR}/SE3Tools.h"
    "${MATH_ROOT_DIR}/UnscentedKalmanFilter.h")

add_library(Math${TARGET_SUFFIX} OBJECT ${MATH_SOURCES})
target_sources(Math${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:MathBase${TARGET_SUFFIX}> $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}>)
set_property(TARGET Math${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET Math${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
else()
  target_compile_options(Math${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
endif()
target_link_libraries(Math${TARGET_SUFFIX} PUBLIC MathBase${TARGET_SUFFIX})
target_link_libraries(Math${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_link_libraries(Math${TARGET_SUFFIX} PRIVATE Flags::Default)
target_include_directories(Math${TARGET_SUFFIX} PUBLIC "${MATH_ROOT_DIR}/..")
source_group(TREE "${MATH_ROOT_DIR}" FILES ${MATH_SOURCES})
