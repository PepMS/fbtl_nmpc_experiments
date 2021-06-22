# Full-Body Torque-Level Nonlinear Model Predictive Control for Aerial Manipulation
This repository contains the needed code to reproduce the experiments presented in the paper [Full-Body Torque-Level Nonlinear Model Predictive Control for Aerial Manipulation](link).

## Software dependencies
### EagleMPC
- Install the [EagleMPC library](https://github.com/PepMS/eagle-mpc) and its dependencies.

:warning: **Crocoddyl version** :warning: To get the same results as in the paper, you should checkout the Crocoddyl repository to [this tag](https://github.com/PepMS/crocoddyl/releases/tag/fbtlnmpc_uam).
### EagleMPC-ROS
The experiments to test the MPC controllers have been run in a simulated environment involving [Gazebo](http://gazebosim.org/) and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). 
Make sure both are installed intro your computer.

Then, you need to clone and download the ROS packages in [EagleMPC-ROS](https://github.com/PepMS/eagle_mpc_ros).
Follow the installation instructions from the repository.

## Experiments
To run the different experiments you first need to clone this repository into your computer.
```console
cd <choose-your-path>
git clone https://github.com/PepMS/fbtl_nmpc_experiments.git
```

### <a name="to"></a> Trajectory optimization
To run a trajectory optimization experiment you need to do 2 steps:
1. Modify the trajectory `.yaml` file with your paths. For example, open the `hexacopter370_flying_arm_3_eagle_catch.yaml` and change the `urdf` and the `follow` fields. Substitute the text between `<>` with the actual paths to the respective libraries.
```yaml
trajectory:
  robot:
    name: "hexacopter_370_flying_arm_3"
    urdf: "<path-to-example-robot-data>/example-robot-data/robots/hexacopter370_description/urdf/hexacopter370_flying_arm_3.urdf"
    follow: "<path-to-ros-ws>/src/eagle_mpc_ros/eagle_mpc_yaml/multicopter/hexacopter370.yaml"
```

2. Execute the selected Python script (with the `display` option in case that you want to visualize this in the Gepetto-Viewer). For example, for the *Eagle's Catch* case:
```
cd <choose-your-path>/fbtl_nmpc_experiments/trajectory-optimization
python3 eagle_catch.py display
```
### <a name="mpc"></a> nonLinear Model Predictive Control
To run an nMPC experiment. For example, the 4-Displacement experiment.
1. Modify the file `mpc/displacement/hexacopter370_flying_arm_3_mpc.yaml`, which is placed inside the same folder as the Python script. Then, substitute the text between `<>` with the actual paths to the respective libraries.
    ```yaml
    trajectory:
      robot:
        name: "hexacopter_370_flying_arm_3"
        urdf: "<path-to-example-robot-data>/example-robot-data/robots/hexacopter370_description/urdf/hexacopter370_flying_arm_3.urdf"
        follow: "<path-to-ros-ws>/src/eagle_mpc_ros/eagle_mpc_yaml/multicopter/hexacopter370.yaml"
    ```
2. Analogously, modify the `yaml` file associated to the trajectory, either `4-displacement` (`hexacopter370_flying_arm_3_displacement.yaml`) or the `eagle_catch` (`hexacopter370_flying_arm_3_eagle_catch_nc.yaml`). These are placed inside the `eagle_mpc_yaml` ROS package.
3. Open the script of the experiment you want to run. For example `mpc/displacement/displacement.py`
4. Set the variables in the section `# -----VARIABLES-----` to match your settings.
    1. `mpcController` allows you to select among the nMPC controllers (Weighted, Rail and Carrot)
    2. `controller_settings_path` is the path of the `.yaml` file contained in the same folder of the Python script
    3. `controller_settings_destination` is the location of the `eagle_mpc_yaml` ROS package
    4. For the disturbance case, you can also set the properties of the simulated disturbance

