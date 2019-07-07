INCLUDS =
LIBS = dh.o
OPTS = -g -O0 -Wall -Wextra -std=c++11
CXX = g++

dh.o:dh.cpp
	${CXX} ${INCLUDES} -c $< ${OPTS}
kinematics:kinematics.cpp dh.o
	${CXX} ${INCLUDES} -o $@ $< ${LIBS} ${OPTS}