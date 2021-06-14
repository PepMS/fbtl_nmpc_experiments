# Full-Body Torque-Level Nonlinear Model Predictive Control for Aerial Manipulation
This repository contains the needed code to reproduce the experiments presented in the paper [Full-Body Torque-Level Nonlinear Model Predictive Control for Aerial Manipulation](link).

## Software dependencies
### EagleMPC
- Install the [EagleMPC library](https://github.com/PepMS/eagle-mpc)
- [Gepetto Viewer](https://github.com/Gepetto/gepetto-viewer-corba) to display the generated trajectories in the [Trajectory generation](#to) section.
### EagleMPC-ROS
The experiments to test the MPC controllers have been run in a simulated environment involving [Gazebo](http://gazebosim.org/) and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). 
Make sure both are installed intro your computer.

Then, you need to clone and download the ROS packages in [EagleMPC-ROS](https://github.com/PepMS/eagle_mpc_ros). 
The other required dependencies are explained in the documentation of this repository.

## Experiments
To run the different experiments you first need to clone this repository into your computer.
```console
cd <choose-your-path>
git clone https://github.com/PepMS/fbtl_nmpc_experiments.git
```

### <a name="to"></a> Trajectory optimization
To run a trajectory optimization experiment, you need to execute the selected Python script (with the `display` option in case that you want to visualize this in the Gepetto-Viewer). For example, for the *Eagle's Catch* case:
```
cd <choose-your-path>/fbtl_nmpc_experiments/trajectory-optimization
python3 eagle_catch.py display
```
### <a name="mpc"></a> nonLinear Model Predictive Control
To run an nMPC experiment.
1. Open the script of the experiment you want to run. For example `mpc/displacement/displacement.py`
2. Set the variables in the section `# -----VARIABLES-----` to match your settings.
2.1 `mpcController` allows you to select among the nMPC controllers (Weighted, Rail and Carrot)
2.2 `controller_settings_path` is the path of the `.yaml` file contained in the same folder of the Python script
2.3 `controller_settings_destination` is the location of the `eagle_mpc_yaml` ROS package
2.4 For the disturbance case, you can also set the properties of the simulated disturbance

