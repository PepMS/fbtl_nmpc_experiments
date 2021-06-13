import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

import example_robot_data
import crocoddyl
import eagle_mpc
from eagle_mpc.utils.visualizer import MulticopterMpcDisplay
from eagle_mpc.utils.tools import saveLogfile
from eagle_mpc.utils.plots import plotTrajectory, plotTrajectoryGroup, showPlots

from tools.oc_box_deployment import boxDeploymentProblem
from tools.display import boxDeploymentGetPayloadTrajectory, boxDeploymentPrepareDisplay

WITHDISPLAY = 'display' in sys.argv

dt = 20  # ms
problem, squash, params = boxDeploymentProblem(dt)
solver = eagle_mpc.SolverSbFDDP(problem, squash)

callbacks = [crocoddyl.CallbackVerbose()]

solver.setCallbacks(callbacks)

t = time.time()
solver.solve([], [], maxiter=500)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

if WITHDISPLAY:
    robot = example_robot_data.load('hexacopter680_flying_arm_2')

    dts = [dt / 1000.0] * len(solver.xs)
    payloads = boxDeploymentGetPayloadTrajectory(robot, solver.xs)
    nRelease = 152
    payloadEnd = payloads[nRelease]
    payloads[nRelease:] = [payloadEnd for _ in range(nRelease)]

    display = MulticopterMpcDisplay(robot, params, frameNames=['flying_arm_2__gripper'], payload='box', cog=True)
    framePoseTrajectory = display.getFramePoseTrajectoryFromSolver(solver)
    cogTrajectory = display.getCogTrajectoryFromSolver(solver)
    boxDeploymentPrepareDisplay(display)

    display.display(solver.xs,
                    solver.us_squash,
                    dts=dts,
                    factor=1,
                    payloads=payloads,
                    se3s=framePoseTrajectory,
                    cogs=cogTrajectory)
