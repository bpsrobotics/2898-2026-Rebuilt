package frc.robot.commands.swerve

import beaverlib.utils.Units.Angular.AngleUnit
import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI

class CardinalAlign(val getTargetAngle: () -> AngleUnit) : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.CARDINAL_ALIGN.ordinal

    private val rotationPID = PIDController(3.0, 0.1, 0.1)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
    }

    override fun initialize() {
        super.initialize()
        rotationPID.reset()
    }

    override fun execute() {
        val targetAngle = getTargetAngle()
        rotationPID.setpoint = targetAngle.asRadians
        omega = rotationPID.calculate(Drivetrain.pose.rotation.radians)
    }
}
