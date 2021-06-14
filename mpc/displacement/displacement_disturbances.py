import shutil

import roslaunch
import rospy

# -----VARIABLES-----
# Controller type
mpc_controller = 'Carrot'  # ['Rail', 'Carrot']

# Disturbances
disturbance_start = 3.8
disturbance_duration = 0.5
disturbance_force = 10

# Paths
controller_settings_path = "/home/pepms/robotics/paper-related/fbtl_nmpc_experiments/mpc/displacement/hexacopter370_flying_arm_3_mpc.yaml"
controller_settings_destination = "/home/pepms/wsros/mpc-ws/src/eagle_mpc_ros/eagle_mpc_yaml/mpc/"
launch_path = "/home/pepms/wsros/mpc-ws/src/eagle_mpc_ros/eagle_mpc_controller/launch/mpc_runner.launch"

# -----SCRIPT-----
time_shutdown = 40
trajectory_name = 'displacement'
shutil.copy(controller_settings_path, controller_settings_destination)

rviz_arg = 'rviz:=true'
mav_arg = 'mav_name:=hexacopter370'
trajectory_arg = 'trajectory_name:=' + trajectory_name
arm_enable_arm_arg = 'arm_enable:=true'
automatic_start_arg = 'automatic_start:=true'
enable_rosbag_arg = 'rosbag:=false'
record_solver_arg = 'record_solver:=false'
controller_arg = 'mpc_type:=' + mpc_controller

disturbance_enable_arg = "disturbance_enable:=true"
disturbance_direction_arg = "disturbance_direcion:=1 1 0"
disturbance_start_arg = "disturbance_start:=" + str(disturbance_start)
disturbance_duration_arg = "disturbance_duration:=" + str(disturbance_duration)
disturbance_force_arg = "disturbance_force:=" + str(disturbance_force)

list_args = []
list_args.append(rviz_arg)
list_args.append(mav_arg)
list_args.append(trajectory_arg)
list_args.append(arm_enable_arm_arg)
list_args.append(automatic_start_arg)
list_args.append(enable_rosbag_arg)
list_args.append(record_solver_arg)
list_args.append(controller_arg)

list_args.append(disturbance_enable_arg)
list_args.append(disturbance_direction_arg)
list_args.append(disturbance_start_arg)
list_args.append(disturbance_duration_arg)
list_args.append(disturbance_force_arg)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

cli_args = [launch_path] + list_args
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()
rospy.sleep(time_shutdown)
parent.shutdown()