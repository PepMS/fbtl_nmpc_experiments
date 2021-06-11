import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time

import pinocchio
import crocoddyl
import example_robot_data

import multicopter_mpc
from multicopter_mpc.utils.visualizer import MulticopterMpcDisplay
from tools.display import monkeyBarPrepareDisplay
from multicopter_mpc.utils.plots import plotTrajectory, plotTrajectoryGroup, showPlots
from multicopter_mpc.utils.tools import saveLogfile


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
                        markevery=20,
                        linewidth=0.75)
    plotTrajectory(forces, time, axs[2], 0, 1, markersize=2, markevery=20, linewidth=0.75)

    # grasp area
    import matplotlib.transforms as mtransforms
    for ax in axs:
        trans = mtransforms.blended_transform_factory(ax.transData, ax.transAxes)
        ax.fill_between(time,
                        0,
                        1,
                        where=np.logical_and(time <= 1.4, time >= 0.0),
                        facecolor='green',
                        alpha=0.2,
                        transform=trans)

    # axis limits
    axs[0].set_ylim(0, 4)
    axs[1].set_ylim(-4, 4)
    axs[2].set_ylim(-0.2, 550)

    # legend
    lRotors = ['FL', 'L', 'RL', 'RR', 'R', 'FR']
    lJoints = ['Shoulder', 'Elbow', 'Wrist']
    axs[0].legend(lRotors, loc=1, ncol=2)
    axs[1].legend(lJoints, loc=1)

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

dt = 5  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
trajectoryName = 'monkey_bar'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")

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
solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)

callbacks = [crocoddyl.CallbackVerbose()]
if WITHLOG:
    WITHDISPLAY = True
    cbLogger = crocoddyl.CallbackLogger()
    callbacks.append(cbLogger)

solver.setCallbacks(callbacks)

t = time.time()
solver.solve([], [], maxiter=400)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

doPlots()

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

if WITHLOG:
    saveLogfile(
        '/home/pepms/robotics/paper-related/full-body-aerial-manipulator/trajectory_optimization/data/monkey_bar.data',
        cbLogger, dt / 1000, solver.us_squash, display.getForceTrajectoryFromSolver(solver))
