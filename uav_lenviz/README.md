# Operating system for an unmanned air vehicle

## Description --> 
This package takes the initial steps in building an operating system for an unmanned air vehicle.

## Directory structure -->

|--- uav_lenviz                             # package name 
|   |--- include
|   |   |--- uav_lenviz                     # header files, if any
|   |--- launch                             # launch files
|   |   |--- launch_uav_1.py                # launch file for drone 1
|   |   |--- launch_uav_2.py                # launch file for drone 2
|   |   |--- launch_uav_3.py                # launch file for drone 3
|   |   |--- launch_uav_4.py                # launch file for drone 4
|   |   |--- launch_uav_5.py                # launch file for drone 5
|   |--- src                                # cpp executables
|   |   |--- drone_operation                # command drone navigation
|   |   |--- mission_planning               # plan drone path
|   |   |--- survey                         # collect drone path
|   |   |--- mux_drone_pos                  # collects all drone positions 
|   |--- CMakeLists.txt                     # compiler instructions
|   |--- package.xml                        # package information
|   |--- README.md                          # package description
