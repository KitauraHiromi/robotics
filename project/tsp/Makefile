INCLUDS =
LIBS = ../../lib/dh.o ../../lib/kinematics.o
OPTS = -g -O0 -Wall -Wextra -std=c++11
GL_OPTS =  -lglut -lGLU -lGL
CXX = g++

robot_pos_gen:robot_pos_gen.cpp ../../lib/dh.o ../../lib/kinematics.o
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS}
view_robot:view_robot.cpp ../../lib/dh.o ../../lib/kinematics.o ../../lib/glutil/glutil.hpp
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS} ${GL_OPTS}
