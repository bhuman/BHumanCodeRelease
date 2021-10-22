set(TESTS_ROOT_DIR "${BHUMAN_PREFIX}/Src")
set(TESTS_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${OS}/Tests/$<CONFIG>")

file(GLOB TESTS_SOURCES
    "${TESTS_ROOT_DIR}/Platform/${OS}/*.cpp" "${TESTS_ROOT_DIR}/Platform/${OS}/*.h" "${TESTS_ROOT_DIR}/Platform/${OS}/*.mm"
    "${TESTS_ROOT_DIR}/Platform/*.cpp" "${TESTS_ROOT_DIR}/Platform/*.h"
    "${TESTS_ROOT_DIR}/Tools/*.cpp" "${TESTS_ROOT_DIR}/Tools/*.h"
    "${TESTS_ROOT_DIR}/Tools/Debugging/TimingManager.cpp" "${TESTS_ROOT_DIR}/Tools/Debugging/TimingManager.h"
    "${TESTS_ROOT_DIR}/Tools/Math/Random.cpp" "${TESTS_ROOT_DIR}/Tools/Math/Random.h"
    "${TESTS_ROOT_DIR}/Tools/Math/RotationMatrix.cpp" "${TESTS_ROOT_DIR}/Tools/Math/RotationMatrix.h"
    "${TESTS_ROOT_DIR}/Tools/Logging/LoggingTools.cpp" "${TESTS_ROOT_DIR}/Tools/Logging/LoggingTools.h"
    "${TESTS_ROOT_DIR}/Tools/MessageQueue/*.cpp" "${TESTS_ROOT_DIR}/Tools/MessageQueue/*.h"
    "${TESTS_ROOT_DIR}/Tools/Module/*.cpp" "${TESTS_ROOT_DIR}/Tools/Module/*.h"
    "${TESTS_ROOT_DIR}/Tools/Streams/*.cpp" "${TESTS_ROOT_DIR}/Tools/Streams/*.h")

set(TESTS_TREE ${TESTS_SOURCES})

file(GLOB_RECURSE TESTS_SOURCES_ADDITIONAL
    "${TESTS_ROOT_DIR}/Utils/Tests/*.cpp" "${TESTS_ROOT_DIR}/Utils/Tests/*.h")

if(APPLE)
  list(REMOVE_ITEM TESTS_SOURCES_ADDITIONAL "${TESTS_ROOT_DIR}/Utils/Tests/Test.cpp")
  list(APPEND TESTS_SOURCES_ADDITIONAL "${TESTS_ROOT_DIR}/Utils/Tests/Test.mm")
endif()

list(APPEND TESTS_SOURCES ${TESTS_SOURCES_ADDITIONAL})

add_executable(Tests MACOSX_BUNDLE ${TESTS_SOURCES})

set_property(TARGET Tests PROPERTY RUNTIME_OUTPUT_DIRECTORY "${TESTS_OUTPUT_DIR}")
set_property(TARGET Tests PROPERTY FOLDER Utils)
set_property(TARGET Tests PROPERTY MACOSX_BUNDLE_INFO_PLIST "${TESTS_ROOT_DIR}/Utils/Tests/Info.plist")
set_property(TARGET Tests PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
set_property(TARGET Tests PROPERTY XCODE_GENERATE_SCHEME ON)
set_property(TARGET Tests PROPERTY XCODE_PRODUCT_TYPE "com.apple.product-type.bundle.unit-test")

target_include_directories(Tests PRIVATE "${TESTS_ROOT_DIR}")

if(APPLE)
  target_include_directories(Tests SYSTEM PRIVATE ${CORE_SERVICES_FRAMEWORK} ${CORE_SERVICES_FRAMEWORK}/Headers)
  target_link_libraries(Tests PRIVATE ${CORE_SERVICES_FRAMEWORK})
  target_link_libraries(Tests PRIVATE ${APP_KIT_FRAMEWORK})
endif()

target_link_libraries(Tests PRIVATE $<$<PLATFORM_ID:Linux>:flite::flite_cmu_us_slt> $<$<PLATFORM_ID:Linux>:flite::flite_usenglish>
    $<$<PLATFORM_ID:Linux>:flite::flite_cmulex> $<$<PLATFORM_ID:Linux>:flite::flite>)
target_link_libraries(Tests PRIVATE $<$<PLATFORM_ID:Linux>:ALSA::ALSA>)
target_link_libraries(Tests PRIVATE $<$<PLATFORM_ID:Linux>:-lpthread>)

target_link_libraries(Tests PRIVATE Eigen::Eigen)
target_link_libraries(Tests PRIVATE GameController::GameController)
target_link_libraries(Tests PRIVATE GTest::GTest)

target_compile_definitions(Tests PRIVATE TARGET_TOOL GTEST_DONT_DEFINE_FAIL GTEST_DONT_DEFINE_TEST GTEST_HAS_TR1_TUPLE=0)

target_link_libraries(Tests PRIVATE Flags::ForDevelop)

source_group(TREE "${TESTS_ROOT_DIR}" FILES ${TESTS_TREE})
source_group(TREE "${TESTS_ROOT_DIR}/Utils/Tests" FILES ${TESTS_SOURCES_ADDITIONAL})
