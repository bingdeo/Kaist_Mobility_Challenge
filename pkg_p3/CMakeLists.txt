cmake_minimum_required(VERSION 3.5)
project(pkg_p3)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Python module install (pkg_p3/ 폴더)
ament_python_install_package(${PROJECT_NAME})

# launch/config/waypoints install
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY waypoints DESTINATION share/${PROJECT_NAME})

# custom msg generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/V2VState.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)

# 실행 엔트리(루트에 있는 'p3' 파일 설치)
install(PROGRAMS
  p3
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
