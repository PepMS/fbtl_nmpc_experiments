import os
import sys
import numpy as np
import copy
import time

import crocoddyl
import example_robot_data
import eagle_mpc

from eagle_mpc.utils.visualizer import MulticopterMpcDisplay
from tools.oc_push_slide import pushSlideCreateProblem
from tools.display import pushSlideDisplay

WITHDISPLAY = 'display' in sys.argv

dt = 20  # ms
robotName = 'hextilt_flying_arm_5'
trajectoryName = 'push_slide'

trajectory = eagle_mpc.Trajectory()
rootPath = os.getcwd()
trajectory.autoSetup(rootPath + "/trajectories_yaml/" + robotName + '_' + trajectoryName + ".yaml")

robot = example_robot_data.load('hextilt_flying_arm_5')
rModel = robot.model
rState = crocoddyl.StateMultibody(rModel)
tauLb = copy.copy(trajectory.squash.s_lb)
tauUb = copy.copy(trajectory.squash.s_ub)
tauLb[-5:] = np.ones(5) * -2
tauUb[-5:] = np.ones(5) * 2
squashModel = crocoddyl.SquashingModelSmoothSat(tauLb, tauUb, 11)
actModel_ = crocoddyl.ActuationModelMultiCopterBase(rState, trajectory.platform_params.tau_f)
actModel = crocoddyl.ActuationSquashingModel(actModel_, squashModel, actModel_.nu)

problem = pushSlideCreateProblem(rModel, rState, actModel, dt)
solver = eagle_mpc.SolverSbFDDP(problem, squashModel)

callbacks = [crocoddyl.CallbackVerbose()]

solver.setCallbacks(callbacks)
t = time.time()
solver.solve([], [], maxiter=100)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

if WITHDISPLAY:
    dts = [dt / 1000.0] * len(solver.xs)
    display = MulticopterMpcDisplay(robot, trajectory.platform_params, frameNames=['flying_arm_5__gripper'])
    pushSlideDisplay(display)

    framePoseTrajectory = display.getFramePoseTrajectoryFromSolver(solver)
    display.display(solver.xs,
                    solver.us_squash,
                    se3s=framePoseTrajectory,
                    dts=dts,
                    factor=1,
                    fs=display.getForceTrajectoryFromSolver(solver))
