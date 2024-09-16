set(OPTIONS_AND_SKILLS_ROOT_DIR "${BHUMAN_PREFIX}/Src/Modules/BehaviorControl/SkillBehaviorControl")

file(GLOB_RECURSE OPTIONS_AND_SKILLS_SOURCES CONFIGURE_DEPENDS
    "${OPTIONS_AND_SKILLS_ROOT_DIR}/Options/*.cpp" "${OPTIONS_AND_SKILLS_ROOT_DIR}/Options/*.h"
    "${OPTIONS_AND_SKILLS_ROOT_DIR}/Skills/*.cpp" "${OPTIONS_AND_SKILLS_ROOT_DIR}/Skills/*.h")

set(OPTIONS_AND_SKILLS_PCHS
    [["SkillBehaviorControl.h"]])

add_library(OptionsAndSkills${TARGET_SUFFIX} OBJECT ${OPTIONS_AND_SKILLS_SOURCES})
set_property(TARGET OptionsAndSkills${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
set_property(TARGET OptionsAndSkills${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(OptionsAndSkills${TARGET_SUFFIX} PRIVATE "${BHUMAN_PREFIX}/Src")
target_include_directories(OptionsAndSkills${TARGET_SUFFIX} PRIVATE "${OPTIONS_AND_SKILLS_ROOT_DIR}")
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE MathBase${TARGET_SUFFIX})
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Streaming${TARGET_SUFFIX})
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Math${TARGET_SUFFIX})
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE RobotParts${TARGET_SUFFIX})
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Debugging${TARGET_SUFFIX})
target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Framework${TARGET_SUFFIX})

if(BUILD_DESKTOP)
  set_property(TARGET OptionsAndSkills${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(OptionsAndSkills${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(OptionsAndSkills${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG> $<$<CONFIG:Release>:-Wno-unused>)
  target_link_libraries(OptionsAndSkills${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()

target_precompile_headers(OptionsAndSkills${TARGET_SUFFIX} PRIVATE ${OPTIONS_AND_SKILLS_PCHS})

if(MSVC)
  target_compile_options(OptionsAndSkills${TARGET_SUFFIX} PUBLIC /Zm200)
endif()

source_group(TREE "${OPTIONS_AND_SKILLS_ROOT_DIR}" FILES ${OPTIONS_AND_SKILLS_SOURCES})
