#Copyright (C) 2016 by Rui Huang
#huangrui@buaa.edu.cn
#
#This file is part of DenseFace3D.
#
#DenseFace3D is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.
#
#DenseFace3D is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License
#along with DenseFace3D.  If not, see <http://www.gnu.org/licenses/>.

#
# This is a CMake makefile.  You can find the cmake utility and
# information about it at http://www.cmake.org
#

cmake_minimum_required(VERSION 2.8.4)

PROJECT(DenseFace3D)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(dlib REQUIRED)
FIND_PACKAGE( Boost 1.40 REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
#include(../../dlib/cmake)
include(${dlib_DIR}/dlib/cmake)
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})

file(GLOB VIRTUAL	*.cpp *.h *.c)
IF(APPLE)
	# we need the X11 OpenGL and not the OSX internal
	include_directories(/usr/X11/include/)
	link_directories(/usr/X11/lib)
ENDIF(APPLE)

# Tell CMake to compile a program.  We do this with the ADD_EXECUTABLE()
# statement which takes the name of the output executable and then a list of
# .cpp files to compile.  Here each example consists of only one .cpp file but
# in general you will make programs that const of many .cpp files.

ADD_EXECUTABLE(denseFace3D ${VIRTUAL})

# Then we tell it to link with dlib.

target_link_libraries(denseFace3D ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} dlib)

IF(APPLE)
	target_link_libraries(denseFace3D GL X11 GLU GLUT dlib)
ENDIF(APPLE)

# Since there are a lot of examples I'm going to use a macro to simply this
# CMakeLists.txt file.  However, usually you will create only one executable in
# your cmake projects and use the syntax shown above.
MACRO(add_example name)
   ADD_EXECUTABLE(${name} ${name}.cpp)
   TARGET_LINK_LIBRARIES(${name} dlib )
ENDMACRO()


