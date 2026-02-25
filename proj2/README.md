# Project 2 (Task 1 + Task 2 + Task 3 UML)

This folder contains:

- `task1.cpp`: IMU read + Madgwick fusion + quaternion output in terminal.
- `task2.cpp`: OpenGL cube viewer driven by live Madgwick quaternion.
- `UML_Task3.md`: simple UML class diagram for Task 3.
- `Makefile`: build and run targets.

## UML Classes (Task 3)

The UML includes these classes:

- `I2CDevice`
- `LSM6DS33`
- `LIS3MDL`
- `MadgwickAHRS`
- `ImuFusionApp`
- `CubeViewer`

Diagram file:

- `UML_Task3.md`

## Prerequisites (Raspberry Pi / Ubuntu)

```bash
sudo apt update
sudo apt install -y g++ freeglut3-dev mesa-utils
```

## Build

From this folder:

```bash
cd project-1-team_baaa/proj2
make all
```

`make all` builds:

- `task1_app`
- `task2_app`

## Run

Run Task 1 (terminal quaternion):

```bash
make task1
```

Run Task 2 (cube viewer):

```bash
make task2
```

Notes:

- `task1` and `task2` use `sudo` by default in the Makefile.
- If you do not want `sudo`, run:

```bash
make RUN_CMD= task1
make RUN_CMD= task2
```

## Useful Make Targets

- `make task1_build` : build only Task 1 executable.
- `make task2_build` : build only Task 2 executable.
- `make clean` : remove built executables.
