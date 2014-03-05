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

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
SET(NEW_ENTITY_CLASS
TaskJointLimitClamping
)

set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)
file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/dyninv/${PROJECT_NAME}")

DYNAMIC_GRAPH_PYTHON_MODULE("sot/dyninv/task_joint_limit_clamping" ${PROJECT_NAME} sot/dyninv/${PROJECT_NAME}/wrap)
#PYTHON_INSTALL("dynamic_graph/sot/dyninv/${PROJECT_NAME}" "__init__.py" "${PYTHON_SITELIB}")

