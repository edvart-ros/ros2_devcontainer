cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(asv_description)
find_package(ament_cmake REQUIRED)

find_package(xacro REQUIRED)

# xacro_add_files(
#   urdf/wamv_base.urdf.xacro
#     INSTALL DESTINATION urdf
# )

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)


ament_package()
