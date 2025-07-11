# k2_nc
GCode parser for sending AS-based linear moves to Kawasaki robots via the KRNX API.

## Download the code
The code is built as a ROS 2 Humble package. Clone into a Humble workspace and build using colcon.
```git clone git@github.com:ham-lab-isu/k2_nc.git```

## Building the code
For compiling the code, use 
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select k2_nc
```