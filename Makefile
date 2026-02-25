CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic

GL_LIBS ?= -lglut -lGL -lGLU
THREAD_LIBS ?= -pthread

COMMON_OBJS = imu_hal.o madgwick_filter.o

.PHONY: all clean

all: imu_quat imu_cube_live cube_quat

imu_quat: imu_quat_main.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(THREAD_LIBS)

imu_cube_live: imu_cube_live.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(THREAD_LIBS) $(GL_LIBS)

cube_quat: cube_quat.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(GL_LIBS)

clean:
	rm -f *.o imu_quat imu_cube_live cube_quat
