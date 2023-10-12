set(QTPROPERTYBROWSER_ROOT_DIR "${BHUMAN_PREFIX}/Util/qtpropertybrowser")

file(GLOB_RECURSE QTPROPERTYBROWSER_SOURCES CONFIGURE_DEPENDS
    "${QTPROPERTYBROWSER_ROOT_DIR}/*.cpp" "${QTPROPERTYBROWSER_ROOT_DIR}/*.h"
    "${QTPROPERTYBROWSER_ROOT_DIR}/*.qrc")

add_library(qtpropertybrowser STATIC ${QTPROPERTYBROWSER_SOURCES})
set_property(TARGET qtpropertybrowser PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET qtpropertybrowser PROPERTY AUTOMOC ON)
set_property(TARGET qtpropertybrowser PROPERTY AUTORCC ON)
set_property(TARGET qtpropertybrowser PROPERTY FOLDER Libs)
target_link_libraries(qtpropertybrowser PUBLIC Qt6::Core Qt6::Gui Qt6::Widgets)
target_include_directories(qtpropertybrowser PRIVATE "${QTPROPERTYBROWSER_ROOT_DIR}")
target_include_directories(qtpropertybrowser SYSTEM INTERFACE "${QTPROPERTYBROWSER_ROOT_DIR}")

target_compile_definitions(qtpropertybrowser PRIVATE $<$<PLATFORM_ID:Windows>:QT_QTPROPERTYBROWSER_EXPORT> $<$<PLATFORM_ID:Windows>:_SCL_SECURE_NO_WARNINGS>)

target_link_libraries(qtpropertybrowser PRIVATE Flags::Default)

source_group(TREE "${QTPROPERTYBROWSER_ROOT_DIR}" FILES ${QTPROPERTYBROWSER_SOURCES})
