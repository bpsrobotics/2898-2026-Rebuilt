package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI

object VisionTurningHandler {
    fun desiredRotation(): AngleUnit {
        val nextFramePos = (Drivetrain.pose.vector2 + Drivetrain.fieldVelocity.vector2 * 0.02)
        SmartDashboard.putString(
            "CurrentFieldPos",
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)).toString(),
        )
        // If on team side, point towards the hub
        if (
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)) ==
                FieldMapREBUILTWelded.getTeamAllianceArea()
        ) {
            return nextFramePos.angleTo(FieldMapREBUILTWelded.teamHub.center) + PI.radians
        }
        // If on neutral size, rotate to cycle

        if (
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)) ==
                FieldMapREBUILTWelded.AllianceArea.Neutral
        ) {
            // If not blocked by hub, simply turn towards team size
            if (
                nextFramePos.y + Drivetrain.Constants.ROBOT_WIDTH.asMeters / 2 <
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight.y ||
                    nextFramePos.y - Drivetrain.Constants.ROBOT_WIDTH.asMeters / 2 >
                        FieldMapREBUILTWelded.teamHub.shape.topRight.y
            ) {
                return Vector2(nextFramePos.x, 0.0)
                    .angleTo(Vector2(FieldMapREBUILTWelded.teamHub.shape.bottomRight.x, 0.0)) +
                    PI.radians
            }
            // If on the bottom half of hub, point towards the bottom right
            if (nextFramePos.y < FieldMapREBUILTWelded.teamHub.center.y) {
                return nextFramePos.angleTo(
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight - Vector2(0.0, 0.1)
                ) + PI.radians
            }
            // If on the top half of hub, point towards the top right
            return nextFramePos.angleTo(
                FieldMapREBUILTWelded.teamHub.shape.topRight + Vector2(0.0, 0.1)
            ) + PI.radians
        }
        // Otherwise, don't rotate
        return Drivetrain.pose.rotation.radians.radians
    }
}
