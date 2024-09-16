set(PLATFORM_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Platform")

if(MACOS AND "${TARGET_SUFFIX}" STREQUAL "")
  set(SOUNDPLAYER_SUFFIX "mm")
else()
  set(SOUNDPLAYER_SUFFIX "cpp")
endif()

set(PLATFORM_SOURCES
    "${PLATFORM_ROOT_DIR}/BHAssert.cpp"
    "${PLATFORM_ROOT_DIR}/BHAssert.h"
    "${PLATFORM_ROOT_DIR}/File.cpp"
    "${PLATFORM_ROOT_DIR}/File.h"
    "${PLATFORM_ROOT_DIR}/Memory.cpp"
    "${PLATFORM_ROOT_DIR}/Memory.h"
    "${PLATFORM_ROOT_DIR}/MemoryMappedFile.cpp"
    "${PLATFORM_ROOT_DIR}/MemoryMappedFile.h"
    "${PLATFORM_ROOT_DIR}/Semaphore.h"
    "${PLATFORM_ROOT_DIR}/SystemCall.cpp"
    "${PLATFORM_ROOT_DIR}/SystemCall.h"
    "${PLATFORM_ROOT_DIR}/Thread.h"
    "${PLATFORM_ROOT_DIR}/Time.cpp"
    "${PLATFORM_ROOT_DIR}/Time.h"
    "${PLATFORM_ROOT_DIR}/${PLATFORM}/Semaphore.cpp"
    "${PLATFORM_ROOT_DIR}/${PLATFORM}/SoundPlayer.${SOUNDPLAYER_SUFFIX}"
    "${PLATFORM_ROOT_DIR}/${PLATFORM}/SoundPlayer.h"
    "${PLATFORM_ROOT_DIR}/${PLATFORM}/Thread.cpp")
if(NOT BUILD_DESKTOP)
  list(APPEND PLATFORM_SOURCES
      "${PLATFORM_ROOT_DIR}/Nao/NaoBHAssert.cpp")
endif()

add_library(Platform${TARGET_SUFFIX} OBJECT ${PLATFORM_SOURCES})
set_property(TARGET Platform${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
if(BUILD_DESKTOP)
  set_property(TARGET Platform${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  if(MACOS)
    target_include_directories(Platform${TARGET_SUFFIX} SYSTEM PRIVATE ${APP_KIT_FRAMEWORK} ${APP_KIT_FRAMEWORK}/Headers)
    target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE ${APP_KIT_FRAMEWORK})
  endif()
  target_link_libraries(Platform${TARGET_SUFFIX} PUBLIC $<$<PLATFORM_ID:Linux>:-lpthread>)
  if(NOT MINIMAL_PROJECT)
    target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE $<$<PLATFORM_ID:Linux>:flite::flite_cmu_us_slt> $<$<PLATFORM_ID:Linux>:flite::flite_usenglish>
        $<$<PLATFORM_ID:Linux>:flite::flite_cmulex> $<$<PLATFORM_ID:Linux>:flite::flite>)
    target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE $<$<PLATFORM_ID:Linux>:ALSA::ALSA>)
    target_compile_definitions(Platform${TARGET_SUFFIX} PRIVATE $<$<PLATFORM_ID:Linux>:HAVE_ALSA>)
  endif()
  target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE $<$<PLATFORM_ID:Windows>:winmm>)
  target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE $<$<PLATFORM_ID:Windows>:ws2_32>)
  target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE Nao::flite::flite_cmu_us_slt Nao::flite::flite_usenglish Nao::flite::flite_cmulex Nao::flite::flite)
  target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE Nao::ALSA::ALSA)
  target_compile_definitions(Platform${TARGET_SUFFIX} PRIVATE HAVE_ALSA)
  target_compile_definitions(Platform${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(Platform${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(Platform${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()
target_include_directories(Platform${TARGET_SUFFIX} PUBLIC "${PLATFORM_ROOT_DIR}/..")
source_group(TREE "${PLATFORM_ROOT_DIR}" FILES ${PLATFORM_SOURCES})
