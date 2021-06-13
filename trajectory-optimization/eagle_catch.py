import sys
import time
import os

import crocoddyl
import eagle_mpc
import example_robot_data

from tools.display import eagleCatchGetPayloadTrajectory, eagleCatchPrepareDisplay
from eagle_mpc.utils.visualizer import MulticopterMpcDisplay

WITHDISPLAY = 'display' in sys.argv

dt = 20  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
trajectoryName = 'eagle_catch'

trajectory = eagle_mpc.Trajectory()

rootPath = os.getcwd()
trajectory.autoSetup(rootPath + "/trajectories_yaml/" + robotName + '_' + trajectoryName + ".yaml")

problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")

solver = eagle_mpc.SolverSbFDDP(problem, trajectory.squash)

callbacks = [crocoddyl.CallbackVerbose()]
solver.setCallbacks(callbacks)
t = time.time()
solver.solve([], [], maxiter=100)
duration = time.time() - t
print("This is the solving time: ", duration)
print("This is the number of iterations: ", solver.iter + 1)
print("This is the second per iter: ", duration / (solver.iter + 1))

if WITHDISPLAY:
    robot = example_robot_data.load(trajectory.robot_model.name)

    dts = [dt / 1000.0] * len(solver.xs)
    payloads = eagleCatchGetPayloadTrajectory(robot, solver.xs)
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
        -0.5380808711051941, -9.278968811035156, 1.9173346757888794, 0.693411648273468, -0.02101491205394268,
        -0.010982625186443329, 0.7201513648033142
    ]
    cameraClose = [
        -0.023411916568875313, -2.4510273933410645, 0.00392531743273139, 0.7553504705429077, -0.00557495653629303,
        -0.004876686725765467, 0.6552791595458984
    ]
    display.robot.viewer.gui.setCameraTransform("crocoddyl", cameraGeneral)

    display.display(solver.xs,
                    solver.us_squash,
                    fs=display.getForceTrajectoryFromSolver(solver),
                    dts=dts,
                    factor=4,
                    payloads=payloads)
