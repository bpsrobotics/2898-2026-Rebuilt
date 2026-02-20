package frc.robot.commands.vision

import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain

class TargetPoseProvider(
    private val center: Vector2,
    private val distance: DistanceUnit,
    private val rotateAround: () -> AngleUnit,
) {
    var angle = Drivetrain.pose.vector2.angleTo(center)

    fun initialize() {
        angle = Drivetrain.pose.vector2.angleTo(center)
    }

    fun getPose(): Pose2d {
        angle += rotateAround()
        return (Vector2(angle) * distance.asMeters + center).toPose2d(angle)
    }

    fun getAngleAndCalculate(): AngleUnit {
        angle += rotateAround()
        return angle
    }
}

fun doCirclePoint(
    point: Vector2,
    distance: DistanceUnit,
    rotateAround: () -> AngleUnit = { 0.radians },
): Command {
    val targetPoseProvider = TargetPoseProvider(point, distance, rotateAround)
    return CircleAlign(
            targetCenter = { point },
            angleProvider = { targetPoseProvider.getAngleAndCalculate() },
            desiredDistance = { distance },
        )
        .beforeStarting(targetPoseProvider::initialize)
        .alongWith(Drivetrain.doEnableVisionOdometry(false))
        .andThen(Drivetrain.doEnableVisionOdometry())
}
