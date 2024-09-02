# Operating system for an unmanned air vehicle

## Description --> 
This package takes the initial steps in building an operating system for an unmanned air vehicle.

## Directory structure -->

|--- uav_lenviz                                         # package name                <br/> 
| &emsp;  |--- include                                                                <br/>
| &emsp;  | &emsp;  |--- uav_lenviz                     # header files, if any        <br/>
| &emsp;  |--- launch                                   # launch files                <br/>
| &emsp;  |  &emsp; |--- launch_uav_1.py                # launch file for drone 1     <br/>
| &emsp;  | &emsp;  |--- launch_uav_2.py                # launch file for drone 2     <br/>
|  &emsp; |  &emsp; |--- launch_uav_3.py                # launch file for drone 3     <br/>
| &emsp;  |  &emsp; |--- launch_uav_4.py                # launch file for drone 4     <br/>
| &emsp;  | &emsp;  |--- launch_uav_5.py                # launch file for drone 5     <br/>
| &emsp;  |--- src                                      # cpp executables             <br/>
| &emsp;  |  &emsp; |--- drone_operation                # command drone navigation    <br/>
| &emsp;  | &emsp;  |--- mission_planning               # plan drone path             <br/>
| &emsp;  |  &emsp; |--- survey                         # collect drone path          <br/>
| &emsp;  |--- CMakeLists.txt                           # compiler instructions       <br/>
| &emsp;  |--- package.xml                              # package information         <br/>
| &emsp;  |--- README.md                                # package description
  
