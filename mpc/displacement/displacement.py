import shutil
import roslaunch

import rospy

# -----VARIABLES-----
# Controller type
mpcController = 'Carrot'  # ['Weighted', 'Rail', 'Carrot']

# Paths
controller_settings_path = "/home/pepms/robotics/paper-related/fbtl_nmpc_experiments/mpc/displacement/hexacopter370_flying_arm_3_mpc.yaml"
controller_settings_destination = "/home/pepms/noetic_ws/mpc_ws/src/eagle_mpc_ros/eagle_mpc_yaml/mpc/"
launchPath = "/home/pepms/noetic_ws/mpc_ws/src/eagle_mpc_ros/eagle_mpc_controller/launch/mpc_runner.launch"

# -----SCRIPT-----
timeShutdown = 30
if mpcController == 'Weighted':
    trajectoryName = 'displacement_w'
else:
    trajectoryName = 'displacement'

shutil.copy(controller_settings_path, controller_settings_destination)

mavArg = 'mav_name:=hexacopter370'
armEnableArmArg = 'arm_enable:=true'
automaticStartArg = 'automatic_start:=true'
enableRosbagArg = 'rosbag:=false'
recordSolverArg = 'record_solver:=false'
record_solver_level_arg = 'record_solver_level=0'

listArgs = []
listArgs.append(mavArg)
listArgs.append(armEnableArmArg)
listArgs.append(automaticStartArg)
listArgs.append(enableRosbagArg)
listArgs.append(recordSolverArg)
listArgs.append(record_solver_level_arg)
listArgs.append('mpc_type:=' + mpcController)
listArgs.append('trajectory_name:=' + trajectoryName)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

cli_args = [launchPath] + listArgs
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()
rospy.sleep(timeShutdown)
parent.shutdown()