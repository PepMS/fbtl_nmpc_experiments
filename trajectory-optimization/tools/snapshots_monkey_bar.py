import pickle
import time

import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay

from display import monkeyBarPrepareDisplay

data = pickle.load(
    open(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/monkey_bar.data',
        'rb'))

dt = 5  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
trajectoryName = 'monkey_bar'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")

robot = example_robot_data.load(trajectory.robot_model.name)
display = MulticopterMpcDisplay(robot, trajectory.platform_params)
# display = MulticopterMpcDisplay(robot, trajectory.platform_params, frameNames=['flying_arm_3__gripper'])
# display.robot.viewer.gui.setFloatProperty(display.forceGroup + '/4', "Size", 0.5)

monkeyBarPrepareDisplay(display, trajectory)

display.thrustRange = 10
dts = [dt / 1000.0] * len(data['xs'])
cameraGeneral = [
    -0.06877129524946213, -4.644031047821045, -0.27974238991737366, 0.7731379866600037, 0.0, 0.0, 0.6336612105369568
]
print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

idxs = [0, 60, 265, 320, -1]

for i, idx in enumerate(idxs):
    rbt = example_robot_data.load(trajectory.robot_model.name)
    rbt.initViewer(windowName="crocoddyl", loadModel=False)
    rbt_name = "robot_" + str(i)
    rbt.loadViewerModel(rootNodeName=rbt_name)
    if i % 2 == 0:
        alpha = 1
    else:
        alpha = 0.4
    rbt.viewer.gui.setFloatProperty("world/" + rbt_name + "/visuals", "Alpha", alpha)
    img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/monkey_bar_" + str(
        i) + ".png"
    rbt.display(data['xs'][idx][:rbt.nq])
    # display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
    time.sleep(1)

img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/monkey_bar_sequence" + ".png"
display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
