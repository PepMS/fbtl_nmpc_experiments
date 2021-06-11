import numpy as np

import pinocchio


def monkeyBarPrepareDisplay(display, trajectory):
    floorOffset = 0.3
    floorPose = pinocchio.SE3ToXYZQUATtuple(pinocchio.SE3(np.identity(3), np.array([0, 0, -floorOffset])))
    display.robot.viewer.gui.applyConfiguration(display.floorGroup + "/flat", floorPose)
    floorColor = [208 / 255, 208 / 255, 225 / 255, 0.3]
    display.robot.viewer.gui.setColorProperty(display.floorGroup + "/flat", "Color1", floorColor)
    display.robot.viewer.gui.setColorProperty(display.floorGroup + "/flat", "Color2", floorColor)

    display.monkeyBarGroup = "world/monkey_bar"
    display.monkeyBarCylinderRadius = 0.01
    display.monkeyBarCylinderVerticalLength = 0.37 + floorOffset
    display.monkeyBarCylinderVerticalName = display.monkeyBarGroup + "/vertical"
    display.monkeyBarCylinderHorizontalLength = 0.37
    display.monkeyBarCylinderHorizontalName = display.monkeyBarGroup + "/horizontal"
    display.monkeyBarSphereSize = 0.02
    display.monkeyBarSphereName = display.monkeyBarGroup + "/handle"
    display.monkeyBarColor = [15. / 255., 88. / 255, 133. / 255., 1.]

    display.robot.viewer.gui.createGroup(display.monkeyBarGroup)
    display.robot.viewer.gui.addCylinder(display.monkeyBarCylinderVerticalName, display.monkeyBarCylinderRadius,
                                         display.monkeyBarCylinderVerticalLength, display.monkeyBarColor)
    display.robot.viewer.gui.addCylinder(display.monkeyBarCylinderHorizontalName, display.monkeyBarCylinderRadius,
                                         display.monkeyBarCylinderHorizontalLength, display.monkeyBarColor)
    display.robot.viewer.gui.addSphere(display.monkeyBarSphereName, display.monkeyBarSphereSize,
                                       display.monkeyBarColor)

    verticalPose = pinocchio.SE3ToXYZQUATtuple(
        pinocchio.SE3(
            np.identity(3),
            np.array([
                0, display.monkeyBarCylinderHorizontalLength, display.monkeyBarCylinderVerticalLength / 2 - floorOffset
            ])))
    display.robot.viewer.gui.applyConfiguration(display.monkeyBarCylinderVerticalName, verticalPose)

    yRotation = np.zeros([3, 3])
    yRotation[0, 0] = 1
    yRotation[1, 2] = -1
    yRotation[2, 1] = 1

    horizontalPose = pinocchio.SE3ToXYZQUATtuple(
        pinocchio.SE3(
            yRotation,
            np.array([
                0, display.monkeyBarCylinderHorizontalLength / 2 + display.monkeyBarCylinderRadius,
                display.monkeyBarCylinderVerticalLength - floorOffset
            ])))
    display.robot.viewer.gui.applyConfiguration(display.monkeyBarCylinderHorizontalName, horizontalPose)

    spherePose = pinocchio.SE3ToXYZQUATtuple(
        pinocchio.SE3(np.identity(3), np.array([0, 0, display.monkeyBarCylinderVerticalLength - floorOffset])))
    display.robot.viewer.gui.applyConfiguration(display.monkeyBarSphereName, spherePose)

    display.robot.viewer.gui.addXYZaxis("world/pose_ee", [0., 204 / 255., 0., 1.], .02, 0.2)
    display.robot.viewer.gui.applyConfiguration(
        "world/pose_ee",
        pinocchio.SE3ToXYZQUAT(
            trajectory.stages[-1].costs.costs["placement_base_link"].cost.reference.placement).tolist())


def eagleCatchGetPayloadTrajectory(robot, xs):
    payloadMs = []
    for x in xs:
        q = x[:robot.nq]
        pinocchio.framesForwardKinematics(robot.model, robot.data, q)
        M_w_link3 = robot.data.oMf[robot.model.getFrameId("flying_arm_3__gripper")]
        cogPayload_w = M_w_link3.act(np.zeros(3))
        M_w_box = pinocchio.SE3(M_w_link3.rotation, cogPayload_w)
        payloadMs.append(pinocchio.SE3ToXYZQUAT(M_w_box).tolist())

    return payloadMs


def eagleCatchPrepareDisplay(display):
    radiusPayload = 0.03
    display.robot.viewer.gui.setFloatProperty(display.forceGroup + '/4', "Size", 5.)
    display.robot.viewer.gui.setFloatProperty(display.payloadGroup, "Radius", radiusPayload)

    floorOffset = radiusPayload
    floorPose = pinocchio.SE3ToXYZQUATtuple(pinocchio.SE3(np.identity(3), np.array([0, 0, -floorOffset])))
    display.robot.viewer.gui.applyConfiguration(display.floorGroup + "/flat", floorPose)
    floorColor = [208 / 255, 208 / 255, 225 / 255, 0.3]
    display.robot.viewer.gui.setColorProperty(display.floorGroup + "/flat", "Color1", floorColor)
    display.robot.viewer.gui.setColorProperty(display.floorGroup + "/flat", "Color2", floorColor)


def pushSlideDisplay(display):
    floorColor = [208 / 255, 208 / 255, 225 / 255, 0.3]
    display.robot.viewer.gui.setColorProperty(display.floorGroup + "/flat", "Color1", floorColor)
    display.robot.viewer.gui.setColorProperty(display.floorGroup + "/flat", "Color2", floorColor)

    trajectoryGroup = "world/trajectory"
    display.robot.viewer.gui.createGroup(trajectoryGroup)
    #WP1
    R1 = pinocchio.AngleAxis(180 * np.pi / 180, np.array([0, 1, 0])).toRotationMatrix()
    R2 = pinocchio.AngleAxis(0 * np.pi / 180, np.array([1, 0, 0])).toRotationMatrix()
    Mwp1 = pinocchio.SE3(np.dot(R1, R2), np.array([0, 0, 0.5]))
    display.robot.viewer.gui.addXYZaxis(trajectoryGroup + "/WP1", [0., 204 / 255., 0., 1.], .02, 0.2)
    display.robot.viewer.gui.applyConfiguration(trajectoryGroup + "/WP1", pinocchio.SE3ToXYZQUATtuple(Mwp1))

    p2 = Mwp1.act(np.array([0, 1, 0]))
    Mwp2 = pinocchio.SE3(np.dot(R1, R2), p2)
    display.robot.viewer.gui.addXYZaxis(trajectoryGroup + "/WP2", [0., 204 / 255., 0., 1.], .02, 0.2)
    display.robot.viewer.gui.applyConfiguration(trajectoryGroup + "/WP2", pinocchio.SE3ToXYZQUATtuple(Mwp2))

    Ms = pushSlideInterpolate(100)
    lineWidth = 2
    lineColor = [255. / 255., 178 / 255., 102. / 255., 1.]
    display.robot.viewer.gui.addCurve(trajectoryGroup + "/trajectory", [np.array([0., 0., 0.]).tolist()] * 2,
                                      lineColor)
    display.robot.viewer.gui.setCurveLineWidth(trajectoryGroup + "/trajectory", lineWidth)
    ps = []
    for M in Ms:
        ps.append(pinocchio.SE3ToXYZQUAT(M)[:3].tolist())

    display.robot.viewer.gui.setCurvePoints(trajectoryGroup + "/trajectory", ps)

    # Remove friction cone frome gripper
    display.activeContacts['6'] = False

    display.robot.viewer.gui.refresh()


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


def boxDeploymentGetPayloadTrajectory(robot, xs):
    payloadMs = []
    for x in xs:
        q = x[:robot.nq]
        pinocchio.framesForwardKinematics(robot.model, robot.data, q)
        M_w_link2 = robot.data.oMf[robot.model.getFrameId("flying_arm_2__gripper")]
        cogPayload_w = M_w_link2.act(np.zeros(3))
        M_w_box = pinocchio.SE3(M_w_link2.rotation, cogPayload_w)
        payloadMs.append(pinocchio.SE3ToXYZQUAT(M_w_box).tolist())

    return payloadMs


def boxDeploymentPrepareDisplay(display):
    # Anochring point
    quatEndEffectorAnchoring = pinocchio.Quaternion(-0.5, 0.5, -0.5, 0.5)
    posEndEffectorAnchoring = np.array([0.0, 0.0, 2.0])
    MEndEffectorAnchoring = pinocchio.SE3(quatEndEffectorAnchoring, posEndEffectorAnchoring)
    display.robot.viewer.gui.addXYZaxis("world/pose_deployment", [0., 204 / 255., 0., 1.], .02, 0.2)
    display.robot.viewer.gui.applyConfiguration("world/pose_deployment",
                                                pinocchio.SE3ToXYZQUATtuple(MEndEffectorAnchoring))

    # Remove contact elements
    display.robot.viewer.gui.setVisibility("world/robot/friction_cone/3/cone", "OFF")
    display.robot.viewer.gui.setVisibility("world/robot/contact_forces/3", "OFF")
    for i in range(4):
        name = "world/robot/friction_cone/3/lines/" + str(i)
        display.robot.viewer.gui.setVisibility(name, "OFF")

    # Floor offset
    floorOffset = 1
    floorPose = pinocchio.SE3ToXYZQUATtuple(pinocchio.SE3(np.identity(3), np.array([0, 0, -floorOffset])))
    display.robot.viewer.gui.applyConfiguration(display.floorGroup + "/flat", floorPose)

    # Add wall
    xWall = display.payloadBoxSize[2] / 2
    pos1 = [xWall, 1.25, 1.75]
    pos2 = [xWall, 1.25, 2.5]
    pos3 = [xWall, -1.25, 2.5]
    pos4 = [xWall, -1.25, 1.75]
    display.robot.viewer.gui.addSquareFace("world/wall", pos1, pos2, pos3, pos4,
                                           [160. / 255, 160 / 255., 160. / 255, 1.])
    display.robot.viewer.gui.setLightingMode("world/wall", "OFF")

    # display.robot.viewer.gui.applyConfiguration(trajectoryGroup + "/WP2", pinocchio.SE3ToXYZQUATtuple(Mwp2))

    # Ms = pushSlideInterpolate(100)
    # lineWidth = 2
    # lineColor = [255. / 255., 178 / 255., 102. / 255., 1.]
    # display.robot.viewer.gui.addCurve(trajectoryGroup + "/trajectory", [np.array([0., 0., 0.]).tolist()] * 2,
    #                                   lineColor)
    # display.robot.viewer.gui.setCurveLineWidth(trajectoryGroup + "/trajectory", lineWidth)
    # ps = []
    # for M in Ms:
    #     ps.append(pinocchio.SE3ToXYZQUAT(M)[:3].tolist())

    # display.robot.viewer.gui.setCurvePoints(trajectoryGroup + "/trajectory", ps)

    # # Remove friction cone frome gripper
    # display.activeContacts['6'] = False

    display.robot.viewer.gui.refresh()