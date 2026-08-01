package frc.robot.commands.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.measure.Angle
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.convert
import kotlin.math.PI

class CardinalAlign(val getTargetAngle: () -> Angle) : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.CARDINAL_ALIGN.ordinal

    private val rotationPID = PIDController(2.0, 0.01, 0.2)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
    }

    override fun initialize() {
        super.initialize()
        rotationPID.reset()
    }

    override fun execute() {
        val targetAngle = getTargetAngle()
        rotationPID.setpoint = targetAngle.convert(edu.wpi.first.units.Units.Radians)
        omega = rotationPID.calculate(Drivetrain.pose.rotation.radians)
    }
}
