set(BUSH_ROOT_DIR "${BHUMAN_PREFIX}/Src")
set(BUSH_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${OS}/bush/$<CONFIG>")

file(GLOB BUSH_SOURCES
    "${BUSH_ROOT_DIR}/Platform/*.cpp" "${BUSH_ROOT_DIR}/Platform/*.h")
file(GLOB_RECURSE BUSH_SOURCES_ADDITIONAL
    "${BUSH_ROOT_DIR}/Utils/bush/*.cpp" "${BUSH_ROOT_DIR}/Utils/bush/*.h"
    "${BUSH_ROOT_DIR}/Tools/Streams/*.cpp" "${BUSH_ROOT_DIR}/Tools/Streams/*.h"
    "${BUSH_ROOT_DIR}/Platform/${OS}/*.cpp" "${BUSH_ROOT_DIR}/Platform/${OS}/*.h" "${BUSH_ROOT_DIR}/Platform/${OS}/*.mm" )
list(APPEND BUSH_SOURCES ${BUSH_SOURCES_ADDITIONAL})
list(APPEND BUSH_SOURCES
    "${BUSH_ROOT_DIR}/Tools/FunctionList.cpp" "${BUSH_ROOT_DIR}/Tools/FunctionList.h"
    "${BUSH_ROOT_DIR}/Tools/AlignedMemory.cpp" "${BUSH_ROOT_DIR}/Tools/AlignedMemory.h"
    "${BUSH_ROOT_DIR}/Utils/bush/bush.qrc")

if(APPLE)
  set(BUSH_ICONS "${BUSH_ROOT_DIR}/Utils/bush/icons/bush.icns")
  list(APPEND BUSH_SOURCES
      "${BUSH_ROOT_DIR}/Controller/Visualization/Helper.mm"
      "${BUSH_ICONS}")
else()
  list(APPEND BUSH_SOURCES "${BUSH_ROOT_DIR}/Utils/bush/bush.rc")
endif()

set(BUSH_TREE "${BUSH_SOURCES}")

if(APPLE)
  set(BUSH_FRAMEWORKS
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtConcurrent.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtCore.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtDBus.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtGui.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtPrintSupport.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtWidgets.framework")

  set(BUSH_PLUGIN_COCOA "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/plugins/platforms/libqcocoa.dylib")
  set(BUSH_PLUGIN_MACSTYLE "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/plugins/styles/libqmacstyle.dylib")
  set(BUSH_PLUGINS "${BUSH_PLUGIN_COCOA}" "${BUSH_PLUGIN_MACSTYLE}")

  list(APPEND BUSH_SOURCES "${BUSH_FRAMEWORKS}" "${BUSH_PLUGINS}")

  set_source_files_properties(${BUSH_ICONS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  set_source_files_properties(${BUSH_FRAMEWORKS} PROPERTIES MACOSX_PACKAGE_LOCATION Frameworks)
  set_source_files_properties(${BUSH_PLUGIN_COCOA} PROPERTIES MACOSX_PACKAGE_LOCATION PlugIns/platforms)
  set_source_files_properties(${BUSH_PLUGIN_MACSTYLE} PROPERTIES MACOSX_PACKAGE_LOCATION PlugIns/styles)

  source_group("Libs" FILES ${BUSH_FRAMEWORKS} ${BUSH_PLUGINS})
endif()

add_executable(bush WIN32 MACOSX_BUNDLE ${BUSH_SOURCES})

set_property(TARGET bush PROPERTY RUNTIME_OUTPUT_DIRECTORY "${BUSH_OUTPUT_DIR}")
set_property(TARGET bush PROPERTY AUTOMOC ON)
set_property(TARGET bush PROPERTY AUTORCC ON)
set_property(TARGET bush PROPERTY FOLDER Utils)
set_property(TARGET bush PROPERTY MACOSX_BUNDLE_INFO_PLIST "${BUSH_ROOT_DIR}/Utils/bush/Info.plist")
set_property(TARGET bush PROPERTY XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/../Frameworks")
set_property(TARGET bush PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
set_property(TARGET bush PROPERTY XCODE_ATTRIBUTE_COPY_PHASE_STRIP "NO")
set_property(TARGET bush PROPERTY XCODE_GENERATE_SCHEME ON)

target_include_directories(bush PRIVATE "${BUSH_ROOT_DIR}")
target_include_directories(bush PRIVATE $<$<PLATFORM_ID:Windows>:${BHUMAN_PREFIX}/Util/Buildchain/Windows/include>)

target_link_libraries(bush PRIVATE Qt5::Concurrent Qt5::Core Qt5::Gui Qt5::Widgets)
target_link_libraries(bush PRIVATE $<$<PLATFORM_ID:Windows>:winmm> $<$<PLATFORM_ID:Windows>:ws2_32>)
target_link_libraries(bush PRIVATE $<$<PLATFORM_ID:Linux>:flite::flite_cmu_us_slt> $<$<PLATFORM_ID:Linux>:flite::flite_usenglish>
    $<$<PLATFORM_ID:Linux>:flite::flite_cmulex> $<$<PLATFORM_ID:Linux>:flite::flite>)
target_link_libraries(bush PRIVATE $<$<PLATFORM_ID:Linux>:ALSA::ALSA>)
target_link_libraries(bush PRIVATE $<$<PLATFORM_ID:Linux>:-lpthread>)

if(APPLE)
  target_include_directories(bush SYSTEM PRIVATE ${CORE_SERVICES_FRAMEWORK} ${CORE_SERVICES_FRAMEWORK}/Headers)
  target_link_libraries(bush PRIVATE ${CORE_SERVICES_FRAMEWORK})

  target_include_directories(bush SYSTEM PRIVATE ${APP_KIT_FRAMEWORK} ${APP_KIT_FRAMEWORK}/Headers)
  target_link_libraries(bush PRIVATE ${APP_KIT_FRAMEWORK})
endif()

target_compile_definitions(bush PRIVATE TARGET_TOOL)

target_link_libraries(bush PRIVATE Flags::Default)

source_group(TREE "${BUSH_ROOT_DIR}" FILES ${BUSH_TREE})

if(WIN32)
  add_custom_command(TARGET bush POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:bush>/platforms"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "$<TARGET_FILE:Qt5::Concurrent>" "$<TARGET_FILE:Qt5::Core>" "$<TARGET_FILE:Qt5::Gui>" "$<TARGET_FILE:Qt5::Widgets>" "$<TARGET_FILE_DIR:bush>"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:Qt5::qwindows>" "$<TARGET_FILE_DIR:bush>/platforms")
endif()
