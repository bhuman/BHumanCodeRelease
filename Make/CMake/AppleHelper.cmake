set(APPLEHELPER_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/AppleHelper")

set(APPLEHELPER_SOURCES
    "${APPLEHELPER_ROOT_DIR}/Helper.mm"
    "${APPLEHELPER_ROOT_DIR}/Helper.h")

add_library(AppleHelper STATIC ${APPLEHELPER_SOURCES})
set_property(TARGET AppleHelper PROPERTY FOLDER Libs)
target_include_directories(AppleHelper PUBLIC "${APPLEHELPER_ROOT_DIR}/..")
target_link_libraries(AppleHelper PUBLIC Qt6::Gui)
target_include_directories(AppleHelper SYSTEM PRIVATE ${APP_KIT_FRAMEWORK} ${APP_KIT_FRAMEWORK}/Headers)
target_link_libraries(AppleHelper PRIVATE ${APP_KIT_FRAMEWORK})
target_link_libraries(AppleHelper PRIVATE Flags::Default)
source_group(TREE "${APPLEHELPER_ROOT_DIR}" FILES ${APPLEHELPER_SOURCES})
