import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

import example_robot_data
import crocoddyl
import multicopter_mpc
from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay
from multicopter_mpc.utils.tools import saveLogfile
from multicopter_mpc.utils.plots import plotTrajectory, plotTrajectoryGroup, showPlots

from tools.oc_box_deployment import boxDeploymentProblem
from tools.display import boxDeploymentGetPayloadTrajectory, boxDeploymentPrepareDisplay


def doPlots():
    us = np.vstack(solver.us_squash).T

    time = np.array([i * dt / 1000.0 for i in range(len(solver.us))])
    cm = 1 / 2.54
    matplotlib.rcParams['font.size'] = 6
    fig, axs = plt.subplots(2, 1, figsize=(8.5 * cm, 5 * cm), sharex=True)
    plotTrajectoryGroup(us, time, axs, [6], markersize=2, markevery=15, linewidth=0.75)

    # grasp area
    import matplotlib.transforms as mtransforms
    for ax in axs:
        trans = mtransforms.blended_transform_factory(ax.transData, ax.transAxes)
        ax.fill_between(time, 0, 1, where=(time <= 3.0), facecolor='green', alpha=0.2, transform=trans)

    # axis limits
    axs[0].set_ylim(0, 12)
    axs[1].set_ylim(-4, 4)

    # legend
    lRotors = ['FL', 'L', 'RL', 'RR', 'R', 'FR']
    lJoints = ['Shoulder', 'Elbow']
    axs[0].legend(lRotors, loc=1, ncol=2)
    axs[1].legend(lJoints, loc=1)

    # axis labels
    axs[0].set_ylabel('Thrust [N]')
    axs[1].set_ylabel('Torque [Nm]')
    axs[1].set_xlabel('Time [s]')

    for ax in axs:
        spine_lw = 0.4
        spine_color = '#696969'
        ax.xaxis.set_tick_params(width=spine_lw, color=spine_color)
        ax.yaxis.set_tick_params(width=spine_lw, color=spine_color)
        for key in ax.spines:
            ax.spines[key].set_linewidth(spine_lw)
            ax.spines[key].set_color(spine_color)
    
    axs[0].get_legend().get_frame().set_linewidth(spine_lw)
    axs[1].get_legend().get_frame().set_linewidth(spine_lw)

    showPlots()


WITHDISPLAY = 'display' in sys.argv
WITHLOG = 'log' in sys.argv

dt = 20  # ms
problem, squash, params = boxDeploymentProblem(dt)
solver = multicopter_mpc.SolverSbFDDP(problem, squash)

callbacks = [crocoddyl.CallbackVerbose()]
if WITHLOG:
    WITHDISPLAY = True
    cbLogger = crocoddyl.CallbackLogger()
    callbacks.append(cbLogger)
solver.setCallbacks(callbacks)

t = time.time()
solver.solve([], [], maxiter=500)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

doPlots()

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

if WITHLOG:
    saveLogfile(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/box_deployment.data',
        cbLogger,
        dt / 1000,
        us_squash=solver.us_squash,
        forces=display.getForceTrajectoryFromSolver(solver),
        frame_poses=framePoseTrajectory,
        cogs=cogTrajectory)
