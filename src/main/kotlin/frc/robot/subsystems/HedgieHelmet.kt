package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.wpilibj2.command.button.Trigger

fun Vector2.crossesX(origin: Vector2, X: Double): Boolean {
    if (origin.x < X && (origin + this).x < X) return false
    if (origin.x > X && (origin + this).x > X) return false
    return true
}

object HedgieHelmet {
    val trenchDriveTrigger = Trigger({ willCollideWithTrench() })

    fun willCollideWithTrench(): Boolean {
        val robotVelocityVector: Vector2 = Drivetrain.robotVelocity.vector2 * 0.1
        val robotPoseVector: Vector2 = Drivetrain.pose.vector2
        val robotWidthVector: Vector2 = Vector2(Drivetrain.Constants.RobotWidth.asMeters, 0.0)

        for (area in
            arrayOf(
                FieldMapREBUILTWelded.RedAllianceAreaLineX,
                FieldMapREBUILTWelded.BlueAllianceAreaLineX,
            )) {
            if (
                (robotVelocityVector + robotWidthVector).crossesX(robotPoseVector, area.asMeters) ||
                    (robotVelocityVector - robotWidthVector).crossesX(
                        robotPoseVector,
                        area.asMeters,
                    )
            )
                return true
        }

        return false
    }
}
