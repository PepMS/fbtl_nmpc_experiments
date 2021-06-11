import pickle
import time

import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay

from display import boxDeploymentPrepareDisplay, boxDeploymentGetPayloadTrajectory

data = pickle.load(
    open(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/box_deployment.data',
        'rb'))

dt = 20  # ms
robotName = 'hexacopter680_flying_arm_2'
trajectoryName = 'hover'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")
robot = example_robot_data.load(robotName)

dts = [dt / 1000.0] * len(data['xs'])
payloads = boxDeploymentGetPayloadTrajectory(robot, data['xs'])
nRelease = 152
payloadEnd = payloads[nRelease]
payloads[nRelease:] = [payloadEnd for _ in range(nRelease)]

display = MulticopterMpcDisplay(robot,
                                trajectory.platform_params,
                                frameNames=['flying_arm_2__gripper'],
                                payload='box',
                                cog=True)
framePoseTrajectory = data['frame_poses']
cogTrajectory = data['cogs']
boxDeploymentPrepareDisplay(display)

cameraGeneral = [
    -0.594990074634552, -5.757094860076904, 2.902766466140747, 0.5861697793006897, 0.0013555892510339618,
    0.008561587892472744, 0.8101418614387512
]
# print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

display.display(data['xs'],
                data['us_squash'],
                dts=dts,
                factor=4,
                payloads=payloads,
                se3s=data['frame_poses'],
                cogs=data['cogs'])
