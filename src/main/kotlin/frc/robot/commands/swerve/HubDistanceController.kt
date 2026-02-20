package frc.robot.commands.swerve

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.Drivetrain

class HubDistanceController(
    private val desiredDistance: () -> DistanceUnit,
    private val moveAround: () -> Double,
) : DriveManager.DriveRequestBase() {
    override val priority = DriverPriority.HUB_DISTANCE.ordinal
    private val distancePID = PIDController(2.0, 0.3, 0.1)

    override fun initialize() {
        super.initialize()
        distancePID.reset()
    }

    override fun execute() {
        val target = FieldMapREBUILTWelded.teamHub.center
        val currentAngleToCenter = Drivetrain.pose.vector2.angleTo(target)
        val distanceSpeed =
            distancePID.calculate(
                Drivetrain.pose.vector2.distance(target) - desiredDistance().asMeters
            )
        val circleSpeed = moveAround()

        vx = circleSpeed * -currentAngleToCenter.sin() + currentAngleToCenter.cos() * distanceSpeed
        vy = circleSpeed * currentAngleToCenter.cos() + currentAngleToCenter.sin() * distanceSpeed
    }
}
