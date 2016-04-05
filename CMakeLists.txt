#
# This is a CMake makefile.  You can find the cmake utility and
# information about it at http://www.cmake.org
#


cmake_minimum_required(VERSION 2.8.4)

PROJECT(virtual)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
file(GLOB VIRTUAL	*.cpp *.h *.c)
include(../../dlib/cmake)

include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})

IF(APPLE)
	# we need the X11 OpenGL and not the OSX internal
	include_directories(/usr/X11/include/)
	link_directories(/usr/X11/lib)
ENDIF(APPLE)

# Tell CMake to compile a program.  We do this with the ADD_EXECUTABLE()
# statement which takes the name of the output executable and then a list of
# .cpp files to compile.  Here each example consists of only one .cpp file but
# in general you will make programs that const of many .cpp files.

ADD_EXECUTABLE(virtual ${VIRTUAL})

# Then we tell it to link with dlib.
#TARGET_LINK_LIBRARIES(virtual dlib)

target_link_libraries(virtual ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} dlib)

IF(APPLE)
	target_link_libraries(virtual GL X11 GLU GLUT dlib)
ENDIF(APPLE)



FIND_PACKAGE( Boost 1.40 REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
# Since there are a lot of examples I'm going to use a macro to simply this
# CMakeLists.txt file.  However, usually you will create only one executable in
# your cmake projects and use the syntax shown above.
MACRO(add_example name)
   ADD_EXECUTABLE(${name} ${name}.cpp)
   TARGET_LINK_LIBRARIES(${name} dlib )
ENDMACRO()


