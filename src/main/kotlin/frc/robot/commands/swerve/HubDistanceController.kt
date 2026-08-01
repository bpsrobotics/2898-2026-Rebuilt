package frc.robot.commands.swerve

import frc.robot.utils.fieldmap.FieldMapREBUILTWelded
import frc.robot.utils.asMeters
import frc.robot.utils.geometry.vector2
import frc.robot.utils.convert
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.subsystems.Drivetrain
import kotlin.math.cos
import kotlin.math.sin

class HubDistanceController(
    private val desiredDistance: () -> Distance,
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

        val angleRadians = currentAngleToCenter.convert(Units.Radians)
        vx = circleSpeed * -sin(angleRadians) + cos(angleRadians) * distanceSpeed
        vy = circleSpeed * cos(angleRadians) + sin(angleRadians) * distanceSpeed
    }
}
