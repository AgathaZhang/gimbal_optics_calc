cmake_minimum_required(VERSION 3.0.2)
project(gimbal_optics_calc)

# add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-g) # 设置编译类型为 Debug，并添加调试信息
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gimbal_optics_calc
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========= library（可选，但推荐：把算法编成库，node 链接它） =========
add_library(${PROJECT_NAME}
  src/gimbal_optics_calc.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME} yaml-cpp
  ${PROJECT_NAME}_node yaml-cpp
)

# ========= node executable =========
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ========= install（可选） =========
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
