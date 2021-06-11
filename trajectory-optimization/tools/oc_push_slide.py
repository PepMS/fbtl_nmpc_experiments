import numpy as np

import pinocchio
import crocoddyl


def pushSlideCreateProblem(rModel, rState, actModel, dt):
    tPS = 2000
    nPS = int(tPS / dt) + 1

    R1 = pinocchio.AngleAxis(180 * np.pi / 180, np.array([0, 1, 0])).toRotationMatrix()
    R2 = pinocchio.AngleAxis(0 * np.pi / 180, np.array([1, 0, 0])).toRotationMatrix()
    Mwp1 = pinocchio.SE3(np.dot(R1, R2), np.array([0, 0, 0.5]))

    p2 = Mwp1.act(np.array([0, 1, 0]))
    Mwp2 = pinocchio.SE3(np.dot(R1, R2), p2)

    alphas = np.linspace(0, 1, nPS)
    Ms = []
    for alpha in alphas:
        Ms.append(pinocchio.SE3.Interpolate(Mwp1, Mwp2, alpha))

    intModelsApproach = pushSlideCreateModelsApproach(rModel, rState, actModel, dt)
    intModelPrePS = pushSlideCreateModelsPrePS(rModel, rState, actModel, dt, Mwp1)
    intModelsPSMoving = pushSlideCreateModelsPSMoving(rModel, rState, actModel, dt, Mwp1)
    intModelPSEnd = pushSlideCreateModelsPSEnd(rModel, rState, actModel, dt, Mwp2)

    initialState = rState.zero()
    initialState[2] = 1.5

    problem = crocoddyl.ShootingProblem(initialState, intModelsApproach + [intModelPrePS] + intModelsPSMoving,
                                        intModelPSEnd)
    # problem = crocoddyl.ShootingProblem(initialState, intModelsApproach + [intModelPrePS] + intModelsPSMoving,
    #                                     intModelsPSMoving[-1])
    return problem


def pushSlideCreateModelsApproach(rModel, rState, actModel, dt):
    nArmJoints = rState.nv - 6
    nRotors = 6

    # State regularization
    wBasePos, wBaseOri, wBaseVelLin, wBaseVelAng = 1., 1., 1., 1.
    wArmPos = 1.
    wArmVel = 1.
    weightsStateReg = np.array([wBasePos] * 3 + [wBaseOri] * 3 + [wArmPos] * nArmJoints + [wBaseVelLin] * 3 +
                               [wBaseVelAng] * 3 + [wArmVel] * nArmJoints)
    activationStateReg = crocoddyl.ActivationModelWeightedQuad(weightsStateReg)
    costStateReg = crocoddyl.CostModelState(rState, activationStateReg, rState.zero(), actModel.nu)

    # State limits
    stateB = np.zeros(rState.ndx)
    stateB[6:6 + nArmJoints] = rModel.upperPositionLimit[-nArmJoints:]
    stateB[rState.nv + 6:] = rModel.velocityLimit[-nArmJoints:]
    weightStateLimit = np.array([0.] * 6 + [1.] * nArmJoints + [0.] * 6 + [1.] * nArmJoints)
    activationBoundsStateLimits = crocoddyl.ActivationBounds(-stateB, stateB)
    activationStateLimits = crocoddyl.ActivationModelWeightedQuadraticBarrier(activationBoundsStateLimits,
                                                                              weightStateLimit)
    costStateLimits = crocoddyl.CostModelState(rState, activationStateLimits, rState.zero(), actModel.nu)

    # Control regularization
    wRotors = 1.
    wArm = 4.
    weightControlReg = np.array([wRotors] * nRotors + [wArm] * nArmJoints)
    activationControlReg = crocoddyl.ActivationModelWeightedQuad(weightControlReg)
    costControlReg = crocoddyl.CostModelControl(rState, activationControlReg, np.zeros(actModel.nu))

    costs = crocoddyl.CostModelSum(rState, actModel.nu)
    costs.addCost("reg_state", costStateReg, 1e-2)
    costs.addCost("reg_control", costControlReg, 1e-2)
    costs.addCost("state_limits", costStateLimits, 10)

    difModel = crocoddyl.DifferentialActionModelContactFwdDynamics(rState, actModel,
                                                                   crocoddyl.ContactModelMultiple(rState, actModel.nu),
                                                                   costs, 0, True)

    intModel = crocoddyl.IntegratedActionModelRK4(difModel, dt / 1000.)

    T = 1500

    return [intModel] * int(T / dt)


def pushSlideCreateModelsPrePS(rModel, rState, actModel, dt, wp1):
    nArmJoints = rState.nv - 6
    nRotors = 6

    baseLinkId = rModel.getFrameId("hextilt__base_link")
    gripperId = rModel.getFrameId("flying_arm_5__gripper")

    # State regularization
    wBasePos, wBaseOri, wBaseVelLin, wBaseVelAng = 1., 1., 1., 1.
    wArmPos = 1.
    wArmVel = 1.
    weightsStateReg = np.array([wBasePos] * 3 + [wBaseOri] * 3 + [wArmPos] * nArmJoints + [wBaseVelLin] * 3 +
                               [wBaseVelAng] * 3 + [wArmVel] * nArmJoints)
    activationStateReg = crocoddyl.ActivationModelWeightedQuad(weightsStateReg)
    costStateReg = crocoddyl.CostModelState(rState, activationStateReg, rState.zero(), actModel.nu)

    # State limits
    stateB = np.zeros(rState.ndx)
    stateB[6:6 + nArmJoints] = rModel.upperPositionLimit[-nArmJoints:]
    stateB[rState.nv + 6:] = rModel.velocityLimit[-nArmJoints:]
    weightStateLimit = np.array([0.] * 6 + [1.] * nArmJoints + [0.] * 6 + [1.] * nArmJoints)
    activationBoundsStateLimits = crocoddyl.ActivationBounds(-stateB, stateB)
    activationStateLimits = crocoddyl.ActivationModelWeightedQuadraticBarrier(activationBoundsStateLimits,
                                                                              weightStateLimit)
    costStateLimits = crocoddyl.CostModelState(rState, activationStateLimits, rState.zero(), actModel.nu)

    # Control regularization
    wRotors = 1.
    wArm = 5.
    weightControlReg = np.array([wRotors] * nRotors + [wArm] * nArmJoints)
    activationControlReg = crocoddyl.ActivationModelWeightedQuad(weightControlReg)
    costControlReg = crocoddyl.CostModelControl(rState, activationControlReg, np.zeros(actModel.nu))

    # gripper Position
    framePlacementGripper = crocoddyl.FramePlacement(gripperId, wp1)
    costPlacementGripper = crocoddyl.CostModelFramePlacement(rState, framePlacementGripper, actModel.nu)

    # gripper Motion
    motionGripper = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    frameMotionGripper = crocoddyl.FrameMotion(gripperId, motionGripper)
    costMotionGripper = crocoddyl.CostModelFrameVelocity(rState, frameMotionGripper, actModel.nu)

    # Platform Orientation
    orientation = pinocchio.AngleAxis(-np.pi / 2, np.array([0, 0, 1])).toRotationMatrix()
    wOrientation = np.array([1, 1, 0])
    activationOrientation = crocoddyl.ActivationModelWeightedQuad(wOrientation)
    frameRotation = crocoddyl.FrameRotation(baseLinkId, orientation)
    costRotation = crocoddyl.CostModelFrameRotation(rState, activationOrientation, frameRotation, actModel.nu)

    # Platform height
    activationTranslationPlatform = crocoddyl.ActivationModelWeightedQuad(np.array([0, 0, 1]))
    frameTranslationPlatform = crocoddyl.FrameTranslation(baseLinkId, np.array([0, 0, 0.7]))
    costTranslationPlatform = crocoddyl.CostModelFrameTranslation(rState, activationTranslationPlatform,
                                                                  frameTranslationPlatform)

    # Platform Motion
    motionPlatform = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    frameMotionPlatform = crocoddyl.FrameMotion(baseLinkId, motionPlatform)
    costMotionPlatform = crocoddyl.CostModelFrameVelocity(rState, frameMotionPlatform, actModel.nu)

    costs = crocoddyl.CostModelSum(rState, actModel.nu)
    costs.addCost("reg_state", costStateReg, 1e-2)
    costs.addCost("reg_control", costControlReg, 1e-2)
    costs.addCost("state_limits", costStateLimits, 10)
    costs.addCost("translation_platform", costTranslationPlatform, 100)
    costs.addCost("rotation_platform", costRotation, 1000)
    costs.addCost("motion_platform", costMotionPlatform, 1000)
    costs.addCost("placement_gripper", costPlacementGripper, 2000)
    costs.addCost("motion_gripper", costMotionGripper, 2000)

    difModel = crocoddyl.DifferentialActionModelContactFwdDynamics(rState, actModel,
                                                                   crocoddyl.ContactModelMultiple(rState, actModel.nu),
                                                                   costs, 0, True)

    intModel = crocoddyl.IntegratedActionModelRK4(difModel, dt / 1000.)

    return intModel


def pushSlideCreateModelsPSMoving(rModel, rState, actModel, dt, wp1):
    nArmJoints = rState.nv - 6
    nRotors = 6

    baseLinkId = rModel.getFrameId("hextilt__base_link")
    gripperId = rModel.getFrameId("flying_arm_5__gripper")

    # State regularization
    wBasePos, wBaseOri, wBaseVelLin, wBaseVelAng = 1., 1., 1., 1.
    wArmPos = 1.
    wArmVel = 1.
    weightsStateReg = np.array([wBasePos] * 3 + [wBaseOri] * 3 + [wArmPos] * nArmJoints + [wBaseVelLin] * 3 +
                               [wBaseVelAng] * 3 + [wArmVel] * nArmJoints)
    activationStateReg = crocoddyl.ActivationModelWeightedQuad(weightsStateReg)
    costStateReg = crocoddyl.CostModelState(rState, activationStateReg, rState.zero(), actModel.nu)

    # State limits
    stateB = np.zeros(rState.ndx)
    stateB[6:6 + nArmJoints] = rModel.upperPositionLimit[-nArmJoints:]
    stateB[rState.nv + 6:] = rModel.velocityLimit[-nArmJoints:]
    weightStateLimit = np.array([0.] * 6 + [1.] * nArmJoints + [0.] * 6 + [1.] * nArmJoints)
    activationBoundsStateLimits = crocoddyl.ActivationBounds(-stateB, stateB)
    activationStateLimits = crocoddyl.ActivationModelWeightedQuadraticBarrier(activationBoundsStateLimits,
                                                                              weightStateLimit)
    costStateLimits = crocoddyl.CostModelState(rState, activationStateLimits, rState.zero(), actModel.nu)

    # Control regularization
    wRotors = 2.
    wArm = 100.
    weightControlReg = np.array([wRotors] * nRotors + [wArm] * nArmJoints)
    activationControlReg = crocoddyl.ActivationModelWeightedQuad(weightControlReg)
    costControlReg = crocoddyl.CostModelControl(rState, activationControlReg, np.zeros(actModel.nu))

    # gripper Position
    activationTranslationGripper = crocoddyl.ActivationModelWeightedQuad(np.array([1, 0, 1]))
    frameTranslation = crocoddyl.FrameTranslation(gripperId, wp1.translation)
    costTranslationGripper = crocoddyl.CostModelFrameTranslation(rState, activationTranslationGripper,
                                                                 frameTranslation)

    # gripper Motion
    motionGripper = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    activationMotionGripper = crocoddyl.ActivationModelWeightedQuad(np.array([1, 0, 1, 0, 0, 0]))
    frameMotionGripper = crocoddyl.FrameMotion(gripperId, motionGripper)
    costMotionGripper = crocoddyl.CostModelFrameVelocity(rState, activationMotionGripper, frameMotionGripper)

    # gripper Force
    forceGripper = pinocchio.Force(np.array([0, 0, -8, 0, 0, 0]))
    activationForceGripper = crocoddyl.ActivationModelWeightedQuad(np.array([1, 1, 1]))
    frameForceGripper = crocoddyl.FrameForce(gripperId, forceGripper)
    costForceGripper = crocoddyl.CostModelContactForce(rState, activationForceGripper, frameForceGripper, actModel.nu)

    # Platform Orientation
    orientation = pinocchio.AngleAxis(-np.pi / 2, np.array([0, 0, 1])).toRotationMatrix()
    wOrientation = np.array([1, 1, 0])
    activationOrientation = crocoddyl.ActivationModelWeightedQuad(wOrientation)
    frameRotation = crocoddyl.FrameRotation(baseLinkId, orientation)
    costRotation = crocoddyl.CostModelFrameRotation(rState, activationOrientation, frameRotation, actModel.nu)

    activationTranslationPlatform = crocoddyl.ActivationModelWeightedQuad(np.array([0, 0, 1]))
    frameTranslationPlatform = crocoddyl.FrameTranslation(baseLinkId, np.array([0, 0, 0.7]))
    costTranslationPlatform = crocoddyl.CostModelFrameTranslation(rState, activationTranslationPlatform,
                                                                  frameTranslationPlatform)

    costs = crocoddyl.CostModelSum(rState, actModel.nu)
    costs.addCost("reg_state", costStateReg, 1e-2)
    costs.addCost("reg_control", costControlReg, 1e-2)
    costs.addCost("state_limits", costStateLimits, 10)
    costs.addCost("rotation_platform", costRotation, 1000)
    costs.addCost("translation_platform", costTranslationPlatform, 100)
    costs.addCost("placement_gripper", costTranslationGripper, 1000)
    costs.addCost("motion_gripper", costMotionGripper, 1000)
    # costs.addCost("force_gripper", costForceGripper, 5e-1)

    # Contact
    contactGripper = crocoddyl.ContactModel2D(rState, frameTranslation, actModel.nu, np.array([0, 0]))
    contacts = crocoddyl.ContactModelMultiple(rState, actModel.nu)
    contacts.addContact("contactGripper", contactGripper)

    difModel = crocoddyl.DifferentialActionModelContactFwdDynamics(rState, actModel, contacts, costs, 0, True)

    intModel = crocoddyl.IntegratedActionModelRK4(difModel, dt / 1000.)

    T = 2500

    return [intModel] * int(T / dt)


def pushSlideCreateModelsPSEnd(rModel, rState, actModel, dt, wp2):
    nArmJoints = rState.nv - 6
    nRotors = 6

    baseLinkId = rModel.getFrameId("hextilt__base_link")
    gripperId = rModel.getFrameId("flying_arm_5__gripper")

    # State regularization
    wBasePos, wBaseOri, wBaseVelLin, wBaseVelAng = 1., 1., 1., 1.
    wArmPos = 1.
    wArmVel = 1.
    weightsStateReg = np.array([wBasePos] * 3 + [wBaseOri] * 3 + [wArmPos] * nArmJoints + [wBaseVelLin] * 3 +
                               [wBaseVelAng] * 3 + [wArmVel] * nArmJoints)
    activationStateReg = crocoddyl.ActivationModelWeightedQuad(weightsStateReg)
    costStateReg = crocoddyl.CostModelState(rState, activationStateReg, rState.zero(), actModel.nu)

    # State limits
    stateB = np.zeros(rState.ndx)
    stateB[6:6 + nArmJoints] = rModel.upperPositionLimit[-nArmJoints:]
    stateB[rState.nv + 6:] = rModel.velocityLimit[-nArmJoints:]
    weightStateLimit = np.array([0.] * 6 + [1.] * nArmJoints + [0.] * 6 + [1.] * nArmJoints)
    activationBoundsStateLimits = crocoddyl.ActivationBounds(-stateB, stateB)
    activationStateLimits = crocoddyl.ActivationModelWeightedQuadraticBarrier(activationBoundsStateLimits,
                                                                              weightStateLimit)
    costStateLimits = crocoddyl.CostModelState(rState, activationStateLimits, rState.zero(), actModel.nu)

    # Control regularization
    wRotors = 2.
    wArm = 100.
    weightControlReg = np.array([wRotors] * nRotors + [wArm] * nArmJoints)
    activationControlReg = crocoddyl.ActivationModelWeightedQuad(weightControlReg)
    costControlReg = crocoddyl.CostModelControl(rState, activationControlReg, np.zeros(actModel.nu))

    # gripper Position
    framePlacementGripper = crocoddyl.FramePlacement(gripperId, wp2)
    costPlacementGripper = crocoddyl.CostModelFramePlacement(rState, framePlacementGripper, actModel.nu)

    activationTranslationGripper = crocoddyl.ActivationModelWeightedQuad(np.array([1, 1, 1]))
    frameTranslation = crocoddyl.FrameTranslation(gripperId, wp2.translation)
    costTranslationGripper = crocoddyl.CostModelFrameTranslation(rState, activationTranslationGripper,
                                                                 frameTranslation)

    # gripper Motion
    motionGripper = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    frameMotionGripper = crocoddyl.FrameMotion(gripperId, motionGripper)
    costMotionGripper = crocoddyl.CostModelFrameVelocity(rState, frameMotionGripper, actModel.nu)

    # gripper Force
    forceGripper = pinocchio.Force(np.array([0, 0, -8, 0, 0, 0]))
    activationForceGripper = crocoddyl.ActivationModelWeightedQuad(np.array([0, 0, 1]))
    frameForceGripper = crocoddyl.FrameForce(gripperId, forceGripper)
    costForceGripper = crocoddyl.CostModelContactForce(rState, activationForceGripper, frameForceGripper, actModel.nu)

    # Platform Orientation
    orientation = pinocchio.AngleAxis(-np.pi / 2, np.array([0, 0, 1])).toRotationMatrix()
    wOrientation = np.array([1, 1, 0])
    activationOrientation = crocoddyl.ActivationModelWeightedQuad(wOrientation)
    frameRotation = crocoddyl.FrameRotation(baseLinkId, orientation)
    costRotation = crocoddyl.CostModelFrameRotation(rState, activationOrientation, frameRotation, actModel.nu)

    activationTranslationPlatform = crocoddyl.ActivationModelWeightedQuad(np.array([0, 0, 1]))
    frameTranslationPlatform = crocoddyl.FrameTranslation(baseLinkId, np.array([0, 0, 0.7]))
    costTranslationPlatform = crocoddyl.CostModelFrameTranslation(rState, activationTranslationPlatform,
                                                                  frameTranslationPlatform)

    # Platform Motion
    motionPlatform = pinocchio.Motion(np.array([0., 0., 0., 0., 0., 0.]))
    frameMotionPlatform = crocoddyl.FrameMotion(baseLinkId, motionPlatform)
    costMotionPlatform = crocoddyl.CostModelFrameVelocity(rState, frameMotionPlatform, actModel.nu)

    costs = crocoddyl.CostModelSum(rState, actModel.nu)
    costs.addCost("reg_state", costStateReg, 1e-2)
    costs.addCost("reg_control", costControlReg, 1e-2)
    costs.addCost("state_limits", costStateLimits, 10)
    costs.addCost("rotation_platform", costRotation, 1000)
    costs.addCost("motion_platform", costMotionPlatform, 1000)
    costs.addCost("translation_platform", costTranslationPlatform, 100)
    costs.addCost("placement_gripper", costTranslationGripper, 1000)
    costs.addCost("motion_gripper", costMotionGripper, 1000)
    # costs.addCost("force_gripper", costForceGripper, 5e-1)

    # Contact
    contactGripper = crocoddyl.ContactModel2D(rState, frameTranslation, actModel.nu, np.array([4, 4]))
    contacts = crocoddyl.ContactModelMultiple(rState, actModel.nu)
    contacts.addContact("contactGripper", contactGripper)

    difModel = crocoddyl.DifferentialActionModelContactFwdDynamics(rState, actModel, contacts, costs, 0, True)

    intModel = crocoddyl.IntegratedActionModelRK4(difModel, dt / 1000.)

    return intModel


def pushSlideInterpolate(N):
    R1 = pinocchio.AngleAxis(180 * np.pi / 180, np.array([0, 1, 0])).toRotationMatrix()
    R2 = pinocchio.AngleAxis(0 * np.pi / 180, np.array([1, 0, 0])).toRotationMatrix()
    Mwp1 = pinocchio.SE3(np.dot(R1, R2), np.array([0, 0, 0.5]))

    p2 = Mwp1.act(np.array([0, 1, 0]))
    Mwp2 = pinocchio.SE3(np.dot(R1, R2), p2)

    alphas = np.linspace(0, 1, N)
    Ms = []
    for alpha in alphas:
        Ms.append(pinocchio.SE3.Interpolate(Mwp1, Mwp2, alpha))

    return Ms
