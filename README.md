# MAV Autonomy Package

This package contains the simulation environments and scripts used in my bachelor's thesis - Visually Localizing, Approaching and Carrying Objects using MAVs.

## Requirements

To run the simulations, [MAVROS](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation) is required to be installed .

## Run simulations
Run the main simulation with 

`roslaunch mav_autonomy start_offb.launch`

In the launch file, change the loaded parameters to switch between the controllers.

The simulation of the package carrying is launch by the `load.launch` file

## Collecting data

For running multiple simulations, collecting data and creating plots, the code on the repository [rosbag_data](https://github.com/caio-freitas/rosbag_data)