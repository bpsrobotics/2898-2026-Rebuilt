package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.asAngleUnit
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.engine.utils.Polynomial
import frc.robot.engine.DashboardNumber
import kotlin.math.PI
import kotlin.math.sign

object VisionTurningHandler : SubsystemBase() {
    private val posePublisher: StructPublisher<Pose2d> =
        NetworkTableInstance.getDefault().getStructTopic("TargetPose", Pose2d.struct).publish()

    fun getDesiredTarget(pos: Vector2): Vector2? {
        val forwardPose = pos.toPose2d(0.0.radians)
        val allianceArea: FieldMapREBUILTWelded.AllianceArea =
            FieldMapREBUILTWelded.getPoseAllianceArea(forwardPose)

        // If on team side, point towards the hub
        if (allianceArea == FieldMapREBUILTWelded.getTeamAllianceArea()) {
            return FieldMapREBUILTWelded.teamHub.center
        }
        // If on neutral size, rotate to cycle

        if (allianceArea == FieldMapREBUILTWelded.AllianceArea.Neutral) {
            // If not blocked by hub, simply turn towards team size
            if (
                pos.y + Drivetrain.Constants.ROBOT_WIDTH.asMeters / 2 <
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight.y ||
                    pos.y - Drivetrain.Constants.ROBOT_WIDTH.asMeters / 2 >
                        FieldMapREBUILTWelded.teamHub.shape.topRight.y
            ) {
                return Vector2(pos.x - FieldMapREBUILTWelded.teamHub.shape.bottomRight.x, pos.y)
            }
            // If on the bottom half of hub, point towards the bottom right
            if (pos.y < FieldMapREBUILTWelded.teamHub.center.y) {
                return FieldMapREBUILTWelded.teamHub.shape.bottomRight - Vector2(0.0, 0.1)
            }
            // If on the top half of hub, point towards the top right
            return FieldMapREBUILTWelded.teamHub.shape.topRight + Vector2(0.0, 0.1)
        }
        // Otherwise, don't rotate
        return null
    }

    var targetPos: Vector2? = null
    var nextFramePos = Vector2(0.0, 0.0)
    var goalHoodAngle: AngleUnit = 0.0.radians
    var goalShooterAngle: AngleUnit = 0.0.radians
    var dashboardGoalShooter: Double by DashboardNumber(0.0, "Odometry")
    val canFire: Boolean
        get() =
            goalHoodAngle.asRadians < Shooter.Hood.Constants.TOP_POSITION.asRadians &&
                goalHoodAngle.asRadians > Shooter.Hood.Constants.DOWN_POSITION.asRadians &&
                Shooter.Hood.atSetpoint // &&

    // Shooter.atSpeed

    fun adjustTargetPosition(
        pos: Vector2,
        targetPosition: Vector2,
        velocity: Vector2,
        airTimePolynomial: Polynomial,
    ): Vector2 {
        val airTime = airTimePolynomial.calculate(pos.distance(targetPosition))
        return targetPosition - (velocity * airTime)
    }

    fun getTargetPos(pos: Vector2): Vector2? {
        val desiredTarget = getDesiredTarget(pos)
        if (desiredTarget == null) return desiredTarget
        return adjustTargetPosition(
            pos,
            desiredTarget,
            Drivetrain.fieldVelocity.vector2,
            Polynomial(2.0),
        )
    }

    override fun periodic() {
        nextFramePos = (Drivetrain.pose.vector2 + Drivetrain.fieldVelocity.vector2 * 0.02)
        val nextFramePose =
            nextFramePos.toPose2d(
                Drivetrain.pose.rotation.asAngleUnit +
                    (Drivetrain.fieldVelocity.omegaRadiansPerSecond.radiansPerSecond * 0.02.seconds)
            )
        targetPos = getTargetPos(nextFramePos)

        val directionToHub =
            sign((FieldMapREBUILTWelded.teamHub.center - Drivetrain.pose.vector2).x)

        if (targetPos == null) {
            posePublisher.set(Pose2d())
            goalHoodAngle = 0.radians
            goalShooterAngle = Drivetrain.pose.rotation.asAngleUnit
            dashboardGoalShooter = goalShooterAngle.asRadians
            return
        }
        posePublisher.set(targetPos!!.toPose2d(0.radians))
        goalShooterAngle = nextFramePos.angleTo(targetPos!!) // + (PI / 2).radians

        val allianceArea: FieldMapREBUILTWelded.AllianceArea =
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePose)
        if (
            allianceArea != FieldMapREBUILTWelded.getTeamAllianceArea() &&
                allianceArea != FieldMapREBUILTWelded.AllianceArea.BlueTrench &&
                allianceArea != FieldMapREBUILTWelded.AllianceArea.RedTrench
        ) {
            if ((goalShooterAngle - nextFramePose.rotation.asAngleUnit).asRadians > PI) {
                goalHoodAngle += PI.radians
                goalHoodAngle = 0.0.radians
            }
            goalHoodAngle = Shooter.Hood.Constants.TOP_POSITION
        } else {
            goalHoodAngle =
                Shooter.Hood.Constants.kinematics
                    .calculate(nextFramePos.distance(targetPos!!))
                    .radians
            if (sign((nextFramePos - Drivetrain.pose.vector2).x) == directionToHub) {
                return
            }
        }

        dashboardGoalShooter = goalShooterAngle.asRadians
    }
}
