package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI

object VisionTurningHandler {
    private val rotationPID = PIDController(1.0, 0.0, 0.1)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
        SmartDashboard.putData(rotationPID)
        rotationPID.setpoint = 0.0
    }

    fun initialize() {
        rotationPID.reset()
    }

    fun desiredRotation(): AngleUnit {
        val nextFramePos = (Drivetrain.pose.vector2 + Drivetrain.fieldVelocity.vector2 * 0.02)
        SmartDashboard.putString(
            "CurrentFieldPos",
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)).toString(),
        )
        if (
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)) ==
                FieldMapREBUILTWelded.getTeamAllianceArea()
        ) {
            return nextFramePos.angleTo(FieldMapREBUILTWelded.teamHub.center)
        }
        if (
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)) ==
                FieldMapREBUILTWelded.AllianceArea.Neutral
        ) {
            if (
                nextFramePos.y + Drivetrain.Constants.ROBOT_WIDTH.asMeters / 2 <
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight.y ||
                    nextFramePos.y - Drivetrain.Constants.ROBOT_WIDTH.asMeters / 2 >
                        FieldMapREBUILTWelded.teamHub.shape.topRight.y
            ) {
                return Vector2(nextFramePos.x, 0.0)
                    .angleTo(Vector2(FieldMapREBUILTWelded.teamHub.shape.bottomRight.x, 0.0))
            }
            if (nextFramePos.y < FieldMapREBUILTWelded.teamHub.center.y) {
                return nextFramePos.angleTo(
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight - Vector2(0.0, 0.1)
                )
            }
            return nextFramePos.angleTo(
                FieldMapREBUILTWelded.teamHub.shape.topRight + Vector2(0.0, 0.1)
            )
        }
        return Drivetrain.pose.rotation.radians.radians
    }

    fun rotationSpeed(): Double {
        SmartDashboard.putNumber(
            "Odometry/DesiredRotation",
            MathUtil.inputModulus(desiredRotation().asRadians, -PI, PI),
        )
        val error =
            MathUtil.inputModulus(
                desiredRotation().asRadians - Drivetrain.pose.rotation.radians,
                -PI,
                PI,
            )
        return rotationPID.calculate(-error)
    }
}
