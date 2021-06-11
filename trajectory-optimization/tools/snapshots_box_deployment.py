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
# Trajectory
trajectoryGroup = "world/trajectory"
display.robot.viewer.gui.createGroup(trajectoryGroup)
lineWidth = 2
lineColor = [255. / 255., 178 / 255., 102. / 255., 1.]
poss = []
for x in data['xs']:
    poss.append(x[:3].tolist())
display.robot.viewer.gui.addCurve(trajectoryGroup + "/trajectory", poss, lineColor)
display.robot.viewer.gui.setCurveLineWidth(trajectoryGroup + "/trajectory", lineWidth)

cameraGeneral = [
    -0.7195021510124207, -5.745929718017578, 2.344416379928589, 0.6466684937477112, 0.03824206069111824,
    0.040219951421022415, 0.7607495188713074
]
# print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

idxs = [35, 55, 75, 120, 200, -1]
alphas = [1, 0.4, 0.4, 0.4, 0.4, 1]

for i, (idx, alpha) in enumerate(zip(idxs, alphas)):
    rbt = example_robot_data.load(trajectory.robot_model.name)
    rbt.model.name = rbt.model.name + "_" + str(i)
    disp = MulticopterMpcDisplay(rbt,
                                 trajectory.platform_params,
                                 frameNames=['flying_arm_2__gripper'],
                                 payload='box',
                                 cog=True)

    disp.payloadGroup = "world/payload_" + str(i)
    disp._addPayload(disp.payload)

    disp.display([data['xs'][idx]], [data['us_squash'][idx]],
                 dts=dts,
                 factor=1,
                 payloads=[payloads[idx]],
                 se3s=[data['frame_poses'][idx]],
                 cogs=[data['cogs'][idx]])
    disp.robot.viewer.gui.setFloatProperty("world/" + disp.robotName, "Alpha", alpha)

    disp.robot.viewer.gui.setVisibility(disp.forceGroup, "OFF")
    disp.robot.viewer.gui.setVisibility(disp.frictionGroup, "OFF")

    display.display([data['xs'][idx]], [data['us_squash'][idx]],
                    dts=dts,
                    factor=1,
                    payloads=[payloads[idx]],
                    se3s=[data['frame_poses'][idx]],
                    cogs=[data['cogs'][idx]])
    # display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
    img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/box_deployment_" + str(
        i) + ".png"
    # time.sleep(1)

img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/box_deployment_sequence.png"
display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
