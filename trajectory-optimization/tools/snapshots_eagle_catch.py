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
    -0.0017251548124477267, -3.0107104778289795, -0.0038703640457242727, 0.74752277135849, 0.0, 0.0, 0.6641485095024109
]
# print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

idxs = [60, 68, 78, 86, 95]
alphas = [1, 0.4, 1, 0.4, 1]
for i, (idx, alpha) in enumerate(zip(idxs, alphas)):
    rbt = example_robot_data.load(trajectory.robot_model.name)
    rbt.model.name = rbt.model.name + "_" + str(i)
    disp = MulticopterMpcDisplay(rbt,
                                 trajectory.platform_params,
                                 frameNames=['flying_arm_3__gripper'],
                                 payload='sphere')

    radiusPayload = 0.03
    disp.payloadGroup = "world/payload_" + str(i)
    disp._addPayload(disp.payload)
    disp.robot.viewer.gui.setFloatProperty(disp.forceGroup + '/4', "Size", 5.)
    disp.robot.viewer.gui.setFloatProperty(disp.payloadGroup, "Radius", radiusPayload)
    disp.robot.viewer.gui.setFloatProperty("world/" + disp.robotName, "Alpha", alpha)
    disp.robot.viewer.gui.setVisibility(disp.forceGroup, "OFF")
    disp.robot.viewer.gui.setVisibility(disp.frictionGroup, "OFF")    

    img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/eagle_catch_" + str(
        i) + ".png"
    display.display([data['xs'][idx]], [data['us_squash'][idx]], [data['forces'][idx]],
                    payloads=[payloads[idx]],
                    dts=dts,
                    factor=1)
    disp.display([data['xs'][idx]], [data['us_squash'][idx]], [data['forces'][idx]],
                 payloads=[payloads[idx]],
                 dts=dts,
                 factor=1)

    for thrust in disp.thrusts:
        name_thrust = disp.thrustGroup + "/" + thrust
        disp.robot.viewer.gui.setFloatProperty(name_thrust, "Radius", 0.006)

    # display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
    rbt.viewer.gui.addSphere("world/sphere", 0.03, display.payloadColor)
    # time.sleep(1)

img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/eagle_catch_sequence" + ".png"
display.robot.viewer.gui.captureFrame("crocoddyl", img_name)