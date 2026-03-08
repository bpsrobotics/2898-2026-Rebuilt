package frc.robot.commands.swerve

import beaverlib.utils.Units.Angular.radians
import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.VisionTurningHandler
import kotlin.math.PI

/**
 * Drive manager that takes control of the rotation of robot to point towards a target location (Hub
 * when on team side, over bump otherwise)
 */
class HubAlign : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.HUB_ALIGN.ordinal

    private val rotationPID = PIDController(1.0, 0.01, 0.2)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
    }

    override fun initialize() {
        super.initialize()
        rotationPID.reset()
    }

    override fun execute() {
        rotationPID.setpoint = VisionTurningHandler.goalShooterAngle.asRadians + (PI)
        omega = rotationPID.calculate(Drivetrain.pose.rotation.radians)
    }
}
