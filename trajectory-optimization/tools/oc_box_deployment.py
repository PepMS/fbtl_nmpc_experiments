import os
import copy
import numpy as np

import pinocchio
import crocoddyl
import example_robot_data
import eagle_mpc


def createModels(squashModel, tauF):
    robot = example_robot_data.load('hexacopter680_flying_arm_2')
    rModel = robot.model
    rState = crocoddyl.StateMultibody(rModel)

    actModel_ = crocoddyl.ActuationModelMultiCopterBase(rState, tauF)
    actModel = crocoddyl.ActuationSquashingModel(actModel_, squashModel, actModel_.nu)

    return {'rModel': rModel, 'rState': rState, 'actModel': actModel}


def createBaseCosts(rState, actModel, wState, wControl, wLimits):
    nRotors = 6

    # State regularization
    wBasePos, wBaseOri, wBaseVelLin, wBaseVelAng = 1., 1., 1., 1.
    wArmPos, wArmVel = 1., 1.
    weightsStateReg = np.array([wBasePos] * 3 + [wBaseOri] * 3 + [wArmPos] * (rState.nv - 6) + [wBaseVelLin] * 3 +
                               [wBaseVelAng] * 3 + [wArmVel] * (rState.nv - 6))
    activationModelStatesReg = crocoddyl.ActivationModelWeightedQuad(weightsStateReg)
    costStatesReg = crocoddyl.CostModelState(rState, activationModelStatesReg, rState.zero(), actModel.nu)

    # State limits
    stateB = np.zeros(rState.ndx)
    stateB[6] = np.pi / 2 + 0.25
    stateB[7] = np.pi / 2 + 0.25
    stateB[14] = 3
    stateB[15] = 3
    weightStateLimits = np.array([0.] * 6 + [1.] * 2 + [0.] * 6 + [1.] * 2)
    activationBoundsStateLimits = crocoddyl.ActivationBounds(-stateB, stateB)
    activationModelStateLimits = crocoddyl.ActivationModelWeightedQuadraticBarrier(activationBoundsStateLimits,
                                                                                   weightStateLimits)
    costStateLimits = crocoddyl.CostModelState(rState, activationModelStateLimits, rState.zero(), actModel.nu)

    # Control regularization
    wRotors = 1.
    wArm = 100.
    weightsControlReg = np.array([wRotors] * nRotors + [wArm] * 2)
    activationModelControlReg = crocoddyl.ActivationModelWeightedQuad(weightsControlReg)
    costControlReg = crocoddyl.CostModelControl(rState, activationModelControlReg, np.zeros(actModel.nu))

    costs = crocoddyl.CostModelSum(rState, actModel.nu)
    costs.addCost("reg_state", costStatesReg, wState)
    costs.addCost("reg_control", costControlReg, wControl)
    costs.addCost("limits_state", costStateLimits, wLimits)

    return costs


def boxDeploymentProblem(dt):
    robotName = 'hexacopter680_flying_arm_2'
    trajectoryName = 'hover'

    trajectory = eagle_mpc.Trajectory()
    rootPath = os.getcwd()
    trajectory.autoSetup(rootPath + "/trajectories_yaml/" + robotName + '_' + trajectoryName + ".yaml")

    tauLb = copy.copy(trajectory.squash.s_lb)
    tauUb = copy.copy(trajectory.squash.s_ub)
    tauLb[-2:] = np.ones(2) * -5
    tauUb[-2:] = np.ones(2) * 5
    squashModel = crocoddyl.SquashingModelSmoothSat(tauLb, tauUb, 8)

    models = {
        'no_load': createModels(squashModel, trajectory.platform_params.tau_f),
        'load': createModels(squashModel, trajectory.platform_params.tau_f)
    }

    models['load']['rModel'].inertias[models['load']['rModel'].getJointId(
        'flying_arm_2__j_link_1_link_2')].mass = 1.16984
    models['load']['rModel'].inertias[models['load']['rModel'].getJointId(
        'flying_arm_2__j_link_1_link_2')].lever = np.array([-0.00262794, -2.51267e-10, 0.178323])
    models['load']['rModel'].inertias[models['load']['rModel'].getJointId(
        'flying_arm_2__j_link_1_link_2')].inertia = np.array([[0.00629506, -4.24279e-12, -0.000205871],
                                                              [-4.24279e-12, 0.00966053, -2.83174e-11],
                                                              [-0.000205871, -2.83174e-11, 0.00715303]])

    intModelsPreAnchoring = boxDeploymentPreAnchoring(models['load']['rModel'], models['load']['rState'],
                                                      models['load']['actModel'], dt)
    intModelsAnchoring = boxDeploymentAnchoring(models['load']['rModel'], models['load']['rState'],
                                                models['load']['actModel'], dt)
    intModelsFlyAway = boxDeploymentFlyAway(models['no_load']['rModel'], models['no_load']['rState'],
                                            models['no_load']['actModel'], dt)
    intModelHover = boxDeploymentHover(models['no_load']['rModel'], models['no_load']['rState'],
                                       models['no_load']['actModel'], dt)
    problem = crocoddyl.ShootingProblem(models['load']['rState'].zero(),
                                        intModelsPreAnchoring + intModelsAnchoring + intModelsFlyAway, intModelHover)

    return problem, squashModel, trajectory.platform_params


def boxDeploymentPreAnchoring(rModel, rState, actModel, dt):

    costs = createBaseCosts(rState, actModel, 5e-2, 5e-2, 100)

    diffModel = crocoddyl.DifferentialActionModelFreeFwdDynamics(rState, actModel, costs)
    intModel = crocoddyl.IntegratedActionModelEuler(diffModel, dt / 1000.)

    t = 2000
    T = int(t / dt) + 1

    return [intModel] * T


def boxDeploymentAnchoring(rModel, rState, actModel, dt):
    quatEndEffectorAnchoring = pinocchio.Quaternion(-0.5, 0.5, -0.5, 0.5)
    posEndEffectorAnchoring = np.array([0.0, 0.0, 2.0])
    MEndEffectorAnchoring = pinocchio.SE3(quatEndEffectorAnchoring, posEndEffectorAnchoring)
    framePlacementEndEffectorAnchoring = crocoddyl.FramePlacement(rModel.getFrameId("flying_arm_2__gripper"),
                                                                  MEndEffectorAnchoring)
    costPlacementEndEffectorAnchoring = crocoddyl.CostModelFramePlacement(rState, framePlacementEndEffectorAnchoring,
                                                                          actModel.nu)

    motionZero = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    frameMotionLink2Anchoring = crocoddyl.FrameMotion(rModel.getFrameId("flying_arm_2__gripper"), motionZero)
    costMotionLink2Anchoring = crocoddyl.CostModelFrameVelocity(rState, frameMotionLink2Anchoring, actModel.nu)

    quatLink1Anchoring = pinocchio.Quaternion(0.5, -0.5, -0.5, 0.5)
    quatLink1Anchoring.normalize()
    frameRotationLink1 = crocoddyl.FrameRotation(rModel.getFrameId("flying_arm_2__link_1"),
                                                 quatLink1Anchoring.toRotationMatrix())
    costOrientationLink1Anchoring = crocoddyl.CostModelFrameRotation(rState, frameRotationLink1, actModel.nu)

    costs = createBaseCosts(rState, actModel, 1e-3, 5e-2, 100)
    costs.addCost("placementEndEffectorAnchoring", costPlacementEndEffectorAnchoring, 2000)
    costs.addCost("motionLink2Anchoring", costMotionLink2Anchoring, 2000)
    costs.addCost("orientationLink1Anchoring", costOrientationLink1Anchoring, 2000)

    diffModel = crocoddyl.DifferentialActionModelFreeFwdDynamics(rState, actModel, costs)
    intModel = crocoddyl.IntegratedActionModelEuler(diffModel, dt / 1000.)

    t = 1000
    T = int(t / dt) + 1

    return [intModel] * T


def boxDeploymentFlyAway(rModel, rState, actModel, dt):
    weightsArmDown = np.array([0] * 6 + [1] * (rState.nv - 6) + [0] * 6 + [1] * (rState.nv - 6))
    activationModelArmDown = crocoddyl.ActivationModelWeightedQuad(weightsArmDown)
    costArmDown = crocoddyl.CostModelState(rState, activationModelArmDown, rState.zero(), actModel.nu)

    costs = createBaseCosts(rState, actModel, 5e-2, 5e-2, 100)
    costs.addCost("arm_down", costArmDown, 1)

    diffModel = crocoddyl.DifferentialActionModelFreeFwdDynamics(rState, actModel, costs)
    intModel = crocoddyl.IntegratedActionModelEuler(diffModel, dt / 1000.)

    t = 2000
    T = int(t / dt) + 1

    return [intModel] * T


def boxDeploymentHover(rModel, rState, actModel, dt):
    weightsArmDown = np.array([0] * 6 + [1] * (rState.nv - 6) + [0] * 6 + [1] * (rState.nv - 6))
    activationModelArmDown = crocoddyl.ActivationModelWeightedQuad(weightsArmDown)
    costArmDown = crocoddyl.CostModelState(rState, activationModelArmDown, rState.zero(), actModel.nu)

    quatLevel = pinocchio.Quaternion(1.0, 0.0, 0.0, 0.0)
    posBaseLinkHover = np.array([-2.0, 0.0, 2.0])
    MBaseLinkHover = pinocchio.SE3(quatLevel, posBaseLinkHover)
    framePlacementBaseLinkHover = crocoddyl.FramePlacement(rModel.getFrameId("hexacopter680__base_link"),
                                                           MBaseLinkHover)
    costFramePlacementBaseLinkHover = crocoddyl.CostModelFramePlacement(rState, framePlacementBaseLinkHover,
                                                                        actModel.nu)

    motionZero = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    frameMotionZeroBaseLink = crocoddyl.FrameMotion(rModel.getFrameId("hexacopter680__base_link"), motionZero)
    costMotionZeroBaseLink = crocoddyl.CostModelFrameVelocity(rState, frameMotionZeroBaseLink, actModel.nu)

    costs = createBaseCosts(rState, actModel, 5e-2, 5e-2, 100)
    costs.addCost("arm_down", costArmDown, 1)
    costs.addCost("base_link_placement", costFramePlacementBaseLinkHover, 2000)
    costs.addCost("base_link_motion", costMotionZeroBaseLink, 2000)

    diffModel = crocoddyl.DifferentialActionModelFreeFwdDynamics(rState, actModel, costs)
    intModel = crocoddyl.IntegratedActionModelEuler(diffModel, dt / 1000.)

    return intModel