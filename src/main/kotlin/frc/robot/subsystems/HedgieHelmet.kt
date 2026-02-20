package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.wpilibj2.command.button.Trigger

fun Vector2.crossesX(origin: Vector2, x: Double): Boolean {
    if (origin.x < x && (origin + this).x < x) return false
    if (origin.x > x && (origin + this).x > x) return false
    return true
}

object HedgieHelmet {
    val trenchDriveTrigger = Trigger { willCollideWithTrench() }

    private fun willCollideWithTrench(): Boolean {
        val robotVelocityVector: Vector2 = Drivetrain.robotVelocity.vector2 * 2.0
        val robotPoseVector: Vector2 = Drivetrain.pose.vector2
        val robotWidthVector = Vector2(Drivetrain.Constants.ROBOT_WIDTH.asMeters, 0.0)

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
