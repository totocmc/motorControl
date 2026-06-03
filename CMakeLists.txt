cmake_minimum_required(VERSION 3.16)
project(motor_control VERSION 0.1.0 LANGUAGES CXX)

add_library(motor_control STATIC
  src/current_loop_controller.cpp
  src/hg7881_motor_driver.cpp
  src/position_current_cascade.cpp
  src/vnh7070as_motor_driver.cpp
)
add_library(motor_control::motor_control ALIAS motor_control)

target_compile_features(motor_control PUBLIC cxx_std_17)
target_include_directories(motor_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

include(GNUInstallDirs)
install(TARGETS motor_control
  EXPORT motor_controlTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(FILES src/motor_control.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY src/motor_control DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT motor_controlTargets
  FILE motor_controlTargets.cmake
  NAMESPACE motor_control::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motor_control
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/motor_controlConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/motor_controlConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/motor_controlConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motor_control
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/motor_controlConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/motor_controlConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motor_control
)
