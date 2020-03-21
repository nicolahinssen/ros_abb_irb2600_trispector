cmake_minimum_required(VERSION 2.8.12)
project(myworkcell_puzzle_ikfast_plugin)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_kdl
  tf2_eigen
  eigen_conversions
)
find_package(LAPACK REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package()

set(IKFAST_LIBRARY_NAME myworkcell_puzzle_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/myworkcell_puzzle_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable)

install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  myworkcell_puzzle_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
