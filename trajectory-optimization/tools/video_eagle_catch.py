import pickle
import time

import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay

from display import eagleCatchGetPayloadTrajectory, eagleCatchPrepareDisplay

data = pickle.load(
    open(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/eagle_catch.data',
        'rb'))

dt = 20  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
trajectoryName = 'eagle_catch'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")

robot = example_robot_data.load(trajectory.robot_model.name)

dts = [dt / 1000.0] * len(data['xs'])
payloads = eagleCatchGetPayloadTrajectory(robot, data['xs'])
tCatch = trajectory.stages[3].t_ini
nCatch = int(tCatch / dt)
payloadInit = [0, 0, 0, 0, 0, 0, 1]
payloads[:nCatch] = [payloadInit for _ in range(nCatch)]

display = MulticopterMpcDisplay(robot,
                                trajectory.platform_params,
                                frameNames=['flying_arm_3__gripper'],
                                payload='sphere')
eagleCatchPrepareDisplay(display)

cameraGeneral = [
    -0.5380808711051941, -9.278968811035156, 1.9173346757888794, 0.693411648273468, -0.02101491205394268,
    -0.010982625186443329, 0.7201513648033142
]
cameraClose = [
    -0.023411916568875313, -2.4510273933410645, 0.00392531743273139, 0.7553504705429077, -0.00557495653629303,
    -0.004876686725765467, 0.6552791595458984
]
# print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

display.display(data['xs'], data['us_squash'], fs=data['forces'], dts=dts, factor=4, payloads=payloads)
