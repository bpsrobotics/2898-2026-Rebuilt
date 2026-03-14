package frc.robot.commands.swerve

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

    companion object {
        const val KP = 2.0
        const val KI = 0.01
        const val KD = 0.2

        fun createRotationPID(): PIDController = PIDController(KP, KI, KD).apply {
            enableContinuousInput(-PI, PI)
        }

        val hubSetpointRadians: Double
            get() = VisionTurningHandler.goalShooterAngle.asRadians + PI
    }

    private val rotationPID = createRotationPID()

    override fun initialize() {
        super.initialize()
        rotationPID.reset()
    }

    override fun execute() {
        rotationPID.setpoint = hubSetpointRadians
        omega = rotationPID.calculate(Drivetrain.pose.rotation.radians)
    }
}
