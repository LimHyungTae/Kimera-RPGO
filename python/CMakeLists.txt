cmake_minimum_required(VERSION 3.14)
project(kimera_rpgo_python)

configure_file(pybind11.CMakeLists.txt.in pybind11-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pybind11-download" OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pybind11-download" OUTPUT_QUIET
)

add_subdirectory(
  ${CMAKE_CURRENT_BINARY_DIR}/pybind11-src ${CMAKE_CURRENT_BINARY_DIR}/pybind11-build
)

pybind11_add_module(
  _kimera_rpgo_bindings bindings/kimera_rpgo_bindings.cpp
)
target_link_libraries(_kimera_rpgo_bindings PRIVATE kimera_rpgo)
target_include_directories(
  _kimera_rpgo_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings
                              ${CMAKE_CURRENT_BINARY_DIR}/bindings
)
