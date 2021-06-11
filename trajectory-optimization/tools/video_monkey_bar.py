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
monkeyBarPrepareDisplay(display, trajectory)

display.thrustRange = 10
dts = [dt / 1000.0] * len(data['xs'])
cameraGeneral = [
    1.0650382041931152, -2.495866060256958, 0.9237831830978394, 0.6351079344749451, 0.022332437336444855,
    0.017950942739844322, 0.7718917727470398
]
print(display.robot.viewer.gui.getCameraTransform("crocoddyl"))
display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

dts = [dt / 1000.0] * len(data['xs'])
display.display(data['xs'], data['us_squash'], dts=dts, factor=4)