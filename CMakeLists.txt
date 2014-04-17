#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Karsten Knese
#  Copyright 2010 CNRS
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(PROJECT_NAME task_joint_limit_clamping)
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS sot_cmake dynamic_graph_bridge jrl_dynamics_urdf roscpp urdf dynamic-graph-python sot-dyninv) 

INCLUDE(${SOT_CMAKE_DIR}/base.cmake)
INCLUDE(${SOT_CMAKE_DIR}/boost.cmake)
INCLUDE(${SOT_CMAKE_DIR}/lapack.cmake)
INCLUDE(${SOT_CMAKE_DIR}/cpack.cmake)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES task_joint_limit_clamping
)

include_directories(include ${catkin_INCLUDE_DIRS})

SET(${PROJECT_NAME}_HEADERS
    include/sot-dyninv/TaskJointLimitClamping.hh
)

ADD_SUBDIRECTORY(src)


