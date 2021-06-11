import pickle
import time

import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay

from display import pushSlideDisplay

data = pickle.load(
    open(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/push_slide.data',
        'rb'))

dt = 20  # ms
robotName = 'hextilt_flying_arm_5'
trajectoryName = 'push_slide'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")

robot = example_robot_data.load('hextilt_flying_arm_5')

display = MulticopterMpcDisplay(robot, trajectory.platform_params, frameNames=['flying_arm_5__gripper'])
# display.robot.viewer.gui.setFloatProperty(display.forceGroup + '/4', "Size", 0.5)
display.robot.viewer.gui.setVisibility("world/floor", "OFF")

pushSlideDisplay(display)

dts = [dt / 1000.0] * len(data['xs'])
cameraGeneral = [
    -2.1677591800689697, 2.3504509925842285, 2.0288290977478027, -0.2602633237838745, 0.4994393587112427,
    0.7349868416786194, -0.3776475489139557
]
# print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

display.display(data['xs'], data['us_squash'], se3s=data['frame_poses'], dts=dts, factor=4, fs=data['forces'])
