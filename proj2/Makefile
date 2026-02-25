CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
GL_LIBS ?= -lglut -lGL -lGLU
RUN_CMD ?= sudo

TASK1_BIN = task1_app
TASK2_BIN = task2_app

.PHONY: all clean task1 task2 task1_build task2_build uml

all: task1_build task2_build

task1_build: $(TASK1_BIN)

task2_build: $(TASK2_BIN)

$(TASK1_BIN): task1.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

$(TASK2_BIN): task2.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(GL_LIBS)

# Run Task 1 (terminal quaternion)
task1: $(TASK1_BIN)
	$(RUN_CMD) ./$(TASK1_BIN)

# Run Task 2 (live cube viewer from Madgwick quaternion)
task2: $(TASK2_BIN)
	$(RUN_CMD) ./$(TASK2_BIN)

uml:
	@echo "Open UML diagram in UML_Task3.md"

clean:
	rm -f $(TASK1_BIN) $(TASK2_BIN)
