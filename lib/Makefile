INCLUDS =
LIBS = dh.o
OPTS = -g -O0 -Wall -Wextra -std=c++11
GL_OPTS =  -lglut -lGLU -lGL
CXX = g++

dh.o:dh.cpp
	${CXX} ${INCLUDES} -c $< ${OPTS}
kinematics.o:kinematics.cpp dh.o
	${CXX} ${INCLUDES} -c $< ${LIBS} ${OPTS}
opengl_test:opengl_test.cpp glutil/glutil.hpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS} ${GL_OPTS}