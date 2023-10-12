find_package(Python3 COMPONENTS Development QUIET)

if(Python3_FOUND)
  set(PYTHON_ROOT_DIR "${BHUMAN_PREFIX}/Src/Libs/Python")
  set(PYTHON_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/Python/$<CONFIG>")

  set(PYTHON_LOGS_SOURCES
      "${PYTHON_ROOT_DIR}/Logs/Frame.cpp"
      "${PYTHON_ROOT_DIR}/Logs/Frame.h"
      "${PYTHON_ROOT_DIR}/Logs/Module.cpp"
      "${PYTHON_ROOT_DIR}/Logs/Log.cpp"
      "${PYTHON_ROOT_DIR}/Logs/Log.h"
      "${PYTHON_ROOT_DIR}/Logs/Types.cpp"
      "${PYTHON_ROOT_DIR}/Logs/Types.h")

  Python3_add_library(PythonLogs MODULE WITH_SOABI ${PYTHON_LOGS_SOURCES})
  set_property(TARGET PythonLogs PROPERTY FOLDER Libs)
  set_property(TARGET PythonLogs PROPERTY LIBRARY_OUTPUT_NAME logs)
  if(DEFINED REAL_VERSION_INFO)
    # We are in a pip build.
    target_compile_definitions(PythonLogs PRIVATE VERSION_INFO=${REAL_VERSION_INFO})
  else()
    set_property(TARGET PythonLogs PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PYTHON_OUTPUT_DIR}")
    set_property(TARGET PythonLogs PROPERTY PDB_OUTPUT_DIRECTORY "${PYTHON_OUTPUT_DIR}")
  endif()
  set_property(TARGET PythonLogs PROPERTY EXCLUDE_FROM_ALL TRUE)
  target_link_libraries(PythonLogs PRIVATE Debugging)
  target_link_libraries(PythonLogs PRIVATE Framework)
  target_link_libraries(PythonLogs PRIVATE Math)
  target_link_libraries(PythonLogs PRIVATE Platform)
  target_link_libraries(PythonLogs PRIVATE Streaming)
  target_link_libraries(PythonLogs PRIVATE snappy::snappy)
  target_include_directories(PythonLogs SYSTEM PRIVATE "${BHUMAN_PREFIX}/Util/pybind11/include")
  target_include_directories(PythonLogs PRIVATE "${PYTHON_ROOT_DIR}/..")
  source_group(TREE "${PYTHON_ROOT_DIR}/Logs" FILES ${PYTHON_LOGS_SOURCES})

  set(PYTHON_CONTROLLER_SOURCES
      "${PYTHON_ROOT_DIR}/Controller/Controller.cpp"
      "${PYTHON_ROOT_DIR}/Controller/Controller.h"
      "${PYTHON_ROOT_DIR}/Controller/Module.cpp"
      "${PYTHON_ROOT_DIR}/Controller/PythonConsole.cpp"
      "${PYTHON_ROOT_DIR}/Controller/PythonConsole.h"
      "${PYTHON_ROOT_DIR}/Controller/PythonRobot.h")

  Python3_add_library(PythonController MODULE WITH_SOABI ${PYTHON_CONTROLLER_SOURCES})
  set_property(TARGET PythonController PROPERTY FOLDER Libs)
  set_property(TARGET PythonController PROPERTY LIBRARY_OUTPUT_NAME controller)
  if(DEFINED REAL_VERSION_INFO)
    # We are in a pip build.
    target_compile_definitions(PythonController PRIVATE VERSION_INFO=${REAL_VERSION_INFO})
  else()
    set_property(TARGET PythonController PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PYTHON_OUTPUT_DIR}")
    set_property(TARGET PythonController PROPERTY PDB_OUTPUT_DIRECTORY "${PYTHON_OUTPUT_DIR}")
  endif()
  set_property(TARGET PythonController PROPERTY EXCLUDE_FROM_ALL TRUE)
  target_link_libraries(PythonController PRIVATE B-Human)
  target_link_libraries(PythonController PRIVATE Framework)
  target_link_libraries(PythonController PRIVATE Platform)
  target_link_libraries(PythonController PRIVATE Streaming)
  target_include_directories(PythonController SYSTEM PRIVATE "${BHUMAN_PREFIX}/Util/pybind11/include")
  target_include_directories(PythonController PRIVATE "${PYTHON_ROOT_DIR}/..")
  source_group(TREE "${PYTHON_ROOT_DIR}/Controller" FILES ${PYTHON_CONTROLLER_SOURCES})
endif()
