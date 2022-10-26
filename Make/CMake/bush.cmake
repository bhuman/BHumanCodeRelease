set(BUSH_ROOT_DIR "${BHUMAN_PREFIX}/Src/Apps/bush")
set(BUSH_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${OS}/bush/$<CONFIG>")

file(GLOB_RECURSE BUSH_SOURCES CONFIGURE_DEPENDS
    "${BUSH_ROOT_DIR}/*.cpp" "${BUSH_ROOT_DIR}/*.h")
list(APPEND BUSH_SOURCES
    "${BUSH_ROOT_DIR}/bush.qrc")

if(APPLE)
  set(BUSH_ICONS "${BUSH_ROOT_DIR}/icons/bush.icns")
  list(APPEND BUSH_SOURCES "${BUSH_ICONS}")
else()
  list(APPEND BUSH_SOURCES "${BUSH_ROOT_DIR}/bush.rc")
endif()

set(BUSH_TREE "${BUSH_SOURCES}")

if(APPLE)
  set(BUSH_FRAMEWORKS
      "${SIMROBOT_PREFIX}/Util/qt/${OS}/lib/QtConcurrent.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${OS}/lib/QtCore.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${OS}/lib/QtDBus.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${OS}/lib/QtGui.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${OS}/lib/QtWidgets.framework")

  set(BUSH_PLUGIN_COCOA "${SIMROBOT_PREFIX}/Util/qt/${OS}/plugins/platforms/libqcocoa.dylib")
  set(BUSH_PLUGIN_MACSTYLE "${SIMROBOT_PREFIX}/Util/qt/${OS}/plugins/styles/libqmacstyle.dylib")
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
set_property(TARGET bush PROPERTY FOLDER Apps)
set_property(TARGET bush PROPERTY MACOSX_BUNDLE_INFO_PLIST "${BUSH_ROOT_DIR}/Info.plist")
set_property(TARGET bush PROPERTY XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/../Frameworks")
set_property(TARGET bush PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
set_property(TARGET bush PROPERTY XCODE_ATTRIBUTE_COPY_PHASE_STRIP "NO")
set_property(TARGET bush PROPERTY XCODE_GENERATE_SCHEME ON)

target_include_directories(bush PRIVATE "${BUSH_ROOT_DIR}")

target_link_libraries(bush PRIVATE Qt6::Concurrent Qt6::Core Qt6::Gui Qt6::Widgets)
target_link_libraries(bush PRIVATE AppleHelper)
target_link_libraries(bush PRIVATE Platform)
target_link_libraries(bush PRIVATE Streaming)
target_link_libraries(bush PRIVATE Flags::Default)

source_group(TREE "${BUSH_ROOT_DIR}" FILES ${BUSH_TREE})

if(WIN32)
  add_custom_command(TARGET bush POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:bush>/platforms"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "$<TARGET_FILE:Qt6::Concurrent>" "$<TARGET_FILE:Qt6::Core>" "$<TARGET_FILE:Qt6::Gui>" "$<TARGET_FILE:Qt6::Widgets>" "$<TARGET_FILE_DIR:bush>"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:Qt6::qwindows>" "$<TARGET_FILE_DIR:bush>/platforms")
endif()
