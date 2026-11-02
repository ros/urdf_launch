cmake_minimum_required(VERSION 3.5)
project(urdf_launch)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

# launch extensions ($(xacro-file ...)), registered by the entry point in setup.cfg
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro_file test/test_xacro_file.py)
endif()

ament_package()
