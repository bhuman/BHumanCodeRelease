set(CONFIG_ROOT_DIR "${BHUMAN_PREFIX}/Config")
file(GLOB_RECURSE CONFIG_FILES
    "${CONFIG_ROOT_DIR}/Behavior/*.cfg"
    "${CONFIG_ROOT_DIR}/KickEngine/*.kmc")
file(GLOB_RECURSE CONFIG_FILES_DEFAULT
    "${CONFIG_ROOT_DIR}/*/Default/*.cfg"
    "${CONFIG_ROOT_DIR}/*/Default/*.def"
    "${CONFIG_ROOT_DIR}/NeuralNets/*.cfg")
file(GLOB_RECURSE CONFIG_FILES_ALL
    "${CONFIG_ROOT_DIR}/*.cfg"
    "${CONFIG_ROOT_DIR}/*.def"
    "${CONFIG_ROOT_DIR}/*.ros2*"
    "${CONFIG_ROOT_DIR}/*.rsi2*"
    "${CONFIG_ROOT_DIR}/*.con"
    "${CONFIG_ROOT_DIR}/*.kmc"
    "${CONFIG_ROOT_DIR}/Scenes/Textures/*.png"
    "${CONFIG_ROOT_DIR}/Scenes/Textures/*.jpg"
    "${CONFIG_ROOT_DIR}/Scenes/Textures/*.svg")
list(REMOVE_ITEM CONFIG_FILES_ALL ${CONFIG_FILES} ${CONFIG_FILES_DEFAULT})
list(APPEND CONFIG_FILES ${CONFIG_FILES_DEFAULT} ${CONFIG_FILES_ALL})
add_custom_target(Config SOURCES ${CONFIG_FILES})
source_group(TREE "${CONFIG_ROOT_DIR}" FILES ${CONFIG_FILES})
