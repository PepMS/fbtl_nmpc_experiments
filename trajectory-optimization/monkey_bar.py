import sys
import os
import numpy as np
import time

import pinocchio
import crocoddyl
import example_robot_data

import eagle_mpc
from eagle_mpc.utils.visualizer import MulticopterMpcDisplay
from tools.display import monkeyBarPrepareDisplay

WITHDISPLAY = 'display' in sys.argv

dt = 5  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
trajectoryName = 'monkey_bar'

trajectory = eagle_mpc.Trajectory()
rootPath = os.getcwd()
trajectory.autoSetup(rootPath + "/trajectories_yaml/" + robotName + '_' + trajectoryName + ".yaml")

# Change torque bounds
minTorque = -3
maxTorque = 3
maxThrust = 4
tauLb = np.array([trajectory.platform_params.min_thrust] * trajectory.platform_params.n_rotors + [minTorque] * 3)
tauUb = np.array([maxThrust] * trajectory.platform_params.n_rotors + [maxTorque] * 3)

trajectory.squash.u_lb = tauLb
trajectory.squash.u_ub = tauUb
trajectory.squash.smooth = trajectory.squash.smooth  # recalculate parameter d, since bounds have changed

# Change the contact point
x0 = trajectory.initial_state
rData = trajectory.robot_model.createData()
pinocchio.forwardKinematics(trajectory.robot_model, rData, x0[:trajectory.robot_model.nq])
pinocchio.updateFramePlacements(trajectory.robot_model, rData)
frameTranslation = crocoddyl.FrameTranslation(
    trajectory.robot_model.getFrameId('flying_arm_3__gripper'),
    rData.oMf[trajectory.robot_model.getFrameId('flying_arm_3__gripper')].translation)
trajectory.stages[0].contacts.contacts['end_effector'].contact.reference = frameTranslation
trajectory.stages[0].costs.costs['translation_ee'].cost.reference = frameTranslation

problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelRK4")
solver = eagle_mpc.SolverSbFDDP(problem, trajectory.squash)

callbacks = [crocoddyl.CallbackVerbose()]
solver.setCallbacks(callbacks)

t = time.time()
solver.solve([], [], maxiter=400)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

if WITHDISPLAY:
    robot = example_robot_data.load(trajectory.robot_model.name)
    display = MulticopterMpcDisplay(robot, trajectory.platform_params, frameNames=['flying_arm_3__gripper'])
    display.robot.viewer.gui.setFloatProperty(display.forceGroup + '/4', "Size", 0.5)

    cameraGeneral = [
        -0.06877129524946213, -4.644031047821045, -0.27974238991737366, 0.7731379866600037, 0.0, 0.0,
        0.6336612105369568
    ]
    display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

    monkeyBarPrepareDisplay(display, trajectory)

    display.thrustRange = 10
    dts = [dt / 1000.0] * len(solver.xs)
    display.display(solver.xs, solver.us_squash, fs=display.getForceTrajectoryFromSolver(solver), dts=dts, factor=4)
