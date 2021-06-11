import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import copy
import time

import pinocchio
import crocoddyl
import example_robot_data
import multicopter_mpc

from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay
from multicopter_mpc.utils.plots import plotTrajectory, plotTrajectoryGroup, showPlots
from multicopter_mpc.utils.tools import saveLogfile
from tools.oc_push_slide import pushSlideCreateProblem
from tools.display import pushSlideDisplay


def doPlots():
    us = np.vstack(solver.us_squash).T

    contactForces = [0] * problem.T
    for idx, (d, m) in enumerate(zip(problem.runningDatas, problem.runningModels)):
        for key, contact in d.differential[0].multibody.contacts.contacts.todict().items():
            if m.differential.contacts.contacts[key].active:
                fiMo = pinocchio.SE3(contact.pinocchio.oMi[contact.joint].rotation.T, contact.jMf.translation)
                force = fiMo.actInv(contact.f)
                contactForces[idx] = np.linalg.norm(force)
    forces = np.vstack(contactForces).T

    time = np.array([i * dt / 1000.0 for i in range(len(solver.us))])

    cm = 1 / 2.54
    matplotlib.rcParams['font.size'] = 6
    fig, axs = plt.subplots(3, 1, figsize=(8.5 * cm, 6.5 * cm), sharex=True)
    plotTrajectoryGroup(us,
                        time,
                        axs, [trajectory.platform_params.n_rotors],
                        markersize=2,
                        markevery=15,
                        linewidth=0.75)
    plotTrajectory(forces, time, axs[2], 0, 1, markersize=2, markevery=15, linewidth=0.75)

    # grasp area
    import matplotlib.transforms as mtransforms
    for ax in axs:
        trans = mtransforms.blended_transform_factory(ax.transData, ax.transAxes)
        ax.fill_between(time, 0, 1, where=(time >= 1.5), facecolor='green', alpha=0.2, transform=trans)

    # axis limits
    axs[0].set_ylim(0, 6)
    axs[1].set_ylim(-1, 1)
    axs[2].set_ylim(-0.2, 15)

    # legend
    lRotors = ['FL', 'L', 'RL', 'RR', 'R', 'FR']
    lJoints = ['Shoulder', 'Elbow', 'Wrist-1', 'Wrist-2', 'Wrist-3']
    axs[0].legend(lRotors, loc=3, ncol=2)
    axs[1].legend(lJoints, loc=3, ncol=2)

    # axis labels
    axs[0].set_ylabel('Thrust [N]')
    axs[1].set_ylabel('Torque [Nm]')
    axs[2].set_ylabel('R. Force [N]')
    axs[2].set_xlabel('Time [s]')

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
robotName = 'hextilt_flying_arm_5'
trajectoryName = 'push_slide'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")

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
solver = multicopter_mpc.SolverSbFDDP(problem, squashModel)

callbacks = [crocoddyl.CallbackVerbose()]
if WITHLOG:
    WITHDISPLAY = True
    cbLogger = crocoddyl.CallbackLogger()
    callbacks.append(cbLogger)

solver.setCallbacks(callbacks)
t = time.time()
solver.solve([], [], maxiter=100)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

doPlots()

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

if WITHLOG:
    saveLogfile(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/push_slide.data',
        cbLogger, dt / 1000, solver.us_squash, display.getForceTrajectoryFromSolver(solver), framePoseTrajectory)
