# Copyright 2010, Florent Lamiraux, CNRS
#
# This file is part of sot-core.
# sot-core is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-core is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph-tutorial. If not, see <http://www.gnu.org/licenses/>.

# Libraries DEFINITIONS and INCLUDES

INCLUDE(${SOT_CMAKE_DIR}/python.cmake)

SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# provide path to dependency libraries
LINK_DIRECTORIES(${DYNAMIC_GRAPH_PYTHON_LIBRARY_DIRS} ${PYTHON_LIBRARY_DIRS})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  TaskJointLimitClamping.cpp
  ../include/sot-dyninv/TaskJointLimitClamping.hh
  )

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

SET(NEW_ENTITY_CLASS
  TaskJointLimitClamping
  )
#PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_joint_limit_clamping.py")

DYNAMIC_GRAPH_PYTHON_MODULE("/sot/dyninv/task_joint_limit_clamping" ${LIBRARY_NAME} jl_wrap)

