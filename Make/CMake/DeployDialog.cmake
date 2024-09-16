set(DEPLOY_DIALOG_ROOT_DIR "${BHUMAN_PREFIX}/Src/Apps/DeployDialog")
set(DEPLOY_DIALOG_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/DeployDialog/$<CONFIG>")

file(GLOB_RECURSE DEPLOY_DIALOG_SOURCES CONFIGURE_DEPENDS
    "${DEPLOY_DIALOG_ROOT_DIR}/*.cpp" "${DEPLOY_DIALOG_ROOT_DIR}/*.h")
list(APPEND DEPLOY_DIALOG_SOURCES
    "${DEPLOY_DIALOG_ROOT_DIR}/DeployDialog.qrc")

if(MACOS)
  set(DEPLOY_DIALOG_ICONS "${DEPLOY_DIALOG_ROOT_DIR}/icons/DeployDialog.icns")
  list(APPEND DEPLOY_DIALOG_SOURCES "${DEPLOY_DIALOG_ICONS}")
else()
  list(APPEND DEPLOY_DIALOG_SOURCES "${DEPLOY_DIALOG_ROOT_DIR}/DeployDialog.rc")
endif()

set(DEPLOY_DIALOG_TREE "${DEPLOY_DIALOG_SOURCES}")

if(MACOS)
  set(DEPLOY_DIALOG_FRAMEWORKS
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtConcurrent.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtCore.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtDBus.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtGui.framework"
      "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/lib/QtWidgets.framework")

  set(DEPLOY_DIALOG_PLUGIN_COCOA "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/plugins/platforms/libqcocoa.dylib")
  set(DEPLOY_DIALOG_PLUGIN_MACSTYLE "${SIMROBOT_PREFIX}/Util/qt/${PLATFORM}/plugins/styles/libqmacstyle.dylib")
  set(DEPLOY_DIALOG_PLUGINS "${DEPLOY_DIALOG_PLUGIN_COCOA}" "${DEPLOY_DIALOG_PLUGIN_MACSTYLE}")

  list(APPEND DEPLOY_DIALOG_SOURCES "${DEPLOY_DIALOG_FRAMEWORKS}" "${DEPLOY_DIALOG_PLUGINS}")

  set_source_files_properties(${DEPLOY_DIALOG_ICONS} PROPERTIES
      MACOSX_PACKAGE_LOCATION Resources)
  set_source_files_properties(${DEPLOY_DIALOG_FRAMEWORKS} PROPERTIES
      MACOSX_PACKAGE_LOCATION Frameworks
      XCODE_FILE_ATTRIBUTES "CodeSignOnCopy;RemoveHeadersOnCopy")
  set_source_files_properties(${DEPLOY_DIALOG_PLUGIN_COCOA} PROPERTIES
      MACOSX_PACKAGE_LOCATION PlugIns/platforms XCODE_FILE_ATTRIBUTES "CodeSignOnCopy")
  set_source_files_properties(${DEPLOY_DIALOG_PLUGIN_MACSTYLE} PROPERTIES
      MACOSX_PACKAGE_LOCATION PlugIns/styles XCODE_FILE_ATTRIBUTES "CodeSignOnCopy")

  source_group("Libs" FILES ${DEPLOY_DIALOG_FRAMEWORKS} ${DEPLOY_DIALOG_PLUGINS})
endif()

add_executable(DeployDialog WIN32 MACOSX_BUNDLE ${DEPLOY_DIALOG_SOURCES})

set_property(TARGET DeployDialog PROPERTY RUNTIME_OUTPUT_DIRECTORY "${DEPLOY_DIALOG_OUTPUT_DIR}")
set_property(TARGET DeployDialog PROPERTY AUTOMOC ON)
set_property(TARGET DeployDialog PROPERTY AUTORCC ON)
set_property(TARGET DeployDialog PROPERTY FOLDER Apps)
set_property(TARGET DeployDialog PROPERTY MACOSX_BUNDLE_INFO_PLIST "${DEPLOY_DIALOG_ROOT_DIR}/Info.plist")
set_property(TARGET DeployDialog PROPERTY XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/../Frameworks")
set_property(TARGET DeployDialog PROPERTY XCODE_ATTRIBUTE_COPY_PHASE_STRIP "NO")
set_property(TARGET DeployDialog PROPERTY XCODE_GENERATE_SCHEME ON)
set_property(TARGET DeployDialog PROPERTY XCODE_SCHEME_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/../Common/deployDialog")
set_property(TARGET DeployDialog PROPERTY XCODE_SCHEME_ARGUMENTS "Debug")
set_property(TARGET DeployDialog PROPERTY XCODE_SCHEME_ENVIRONMENT "IDEPreferLogStreaming=YES")

target_include_directories(DeployDialog PRIVATE "${DEPLOY_DIALOG_ROOT_DIR}")

target_link_libraries(DeployDialog PRIVATE Qt6::Concurrent Qt6::Core Qt6::Gui Qt6::Widgets)
target_link_libraries(DeployDialog PRIVATE Platform)
target_link_libraries(DeployDialog PRIVATE Streaming)
target_link_libraries(DeployDialog PRIVATE Flags::Default)

source_group(TREE "${DEPLOY_DIALOG_ROOT_DIR}" FILES ${DEPLOY_DIALOG_TREE})

if(WIN32)
  add_custom_command(TARGET DeployDialog POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:DeployDialog>/platforms"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "$<TARGET_FILE:Qt6::Concurrent>" "$<TARGET_FILE:Qt6::Core>" "$<TARGET_FILE:Qt6::Gui>" "$<TARGET_FILE:Qt6::Widgets>" "$<TARGET_FILE_DIR:DeployDialog>"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:Qt6::qwindows>" "$<TARGET_FILE_DIR:DeployDialog>/platforms")
endif()
