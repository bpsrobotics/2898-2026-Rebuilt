package frc.robot.commands.swerve

import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.VisionTurningHandler

class HubAlign : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.HUB_ALIGN.ordinal

    private val rotationPID = PIDController(3.0, 0.1, 0.1)

    override fun initialize() {
        super.initialize()
        rotationPID.reset()
    }

    override fun execute() {
        val target = VisionTurningHandler.desiredRotation()
        rotationPID.setpoint = target.asRadians
        omega = rotationPID.calculate(Drivetrain.pose.rotation.radians)
    }
}
