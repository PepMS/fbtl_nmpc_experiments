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
    -2.6665053367614746, 1.3970009088516235, 1.1561897993087769, -0.3992060124874115, 0.5157274007797241,
    0.6046692132949829, -0.45720338821411133
]
print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

# idxs = [55, 75, 120, 180]
idxs = [30, 120, 150, -1]
alphas = [1, 0.4, 0.4, 1]

for i, (idx, alpha) in enumerate(zip(idxs, alphas)):
    rbt = example_robot_data.load(trajectory.robot_model.name)
    rbt.model.name = rbt.model.name + "_" + str(i)
    disp = MulticopterMpcDisplay(rbt, trajectory.platform_params, frameNames=['flying_arm_5__gripper'])
    disp.robot.viewer.gui.setFloatProperty("world/" + rbt.model.name, "Alpha", alpha)
    disp.robot.viewer.gui.setVisibility(disp.forceGroup, "OFF")
    disp.robot.viewer.gui.setVisibility(disp.frictionGroup, "OFF")
    disp.robot.viewer.gui.setVisibility(disp.frictionGroup, "OFF")

    # img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/push_slide_" + str(
    #     i) + ".png"
    disp.display([data['xs'][idx]], [data['us_squash'][idx]], [data['forces'][idx]],
                 dts=dts,
                 factor=1,
                 se3s=[data['frame_poses'][idx]])
    display.display([data['xs'][idx]], [data['us_squash'][idx]], [data['forces'][idx]],
                    dts=dts,
                    factor=1,
                    se3s=[data['frame_poses'][idx]])
    display.robot.viewer.gui.setVisibility("world/" + display.robot.model.name, "OFF")

    # display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
    time.sleep(1)

img_name = "/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/snapshots/push_slide_sequence" + ".png"
display.robot.viewer.gui.captureFrame("crocoddyl", img_name)
