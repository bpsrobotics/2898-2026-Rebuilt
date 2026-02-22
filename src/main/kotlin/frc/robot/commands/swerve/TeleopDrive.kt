package frc.robot.commands.swerve

import frc.robot.engine.DashboardNumberPublisher
import frc.robot.subsystems.Drivetrain
import kotlin.math.pow

/**
 * A command that controls the swerve drive using joystick inputs.
 *
 * @param getVForward The x velocity of the robot.
 * @param getVStrafe The y velocity of the robot.
 * @param getOmega The angular velocity of the robot.
 * @param slowMode Double supplier that ranges from 0 (normal top speed) to 1 (very reduced speed)
 * @see Drivetrain
 */
class TeleopDrive(
    private val getVForward: () -> Double,
    private val getVStrafe: () -> Double,
    private val getOmega: () -> Double,
    private val slowMode: () -> Double,
) : DriveManager.DriveRequestBase() {
    override var vx: Double? = 0.0
    override var vy: Double? = 0.0
    override var omega: Double? = 0.0
    override val priority: Int = DriverPriority.BASE_TELEOP.ordinal

    private var forwardVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    private var strafeVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    private var angVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    private var slowModeCalc: Double by DashboardNumberPublisher(0.0, "Teleop/")

    override fun execute() {
        forwardVelocity = getVForward()
        strafeVelocity = getVStrafe()
        angVelocity = getOmega()
        slowModeCalc = slowMode()

        forwardVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.8))
        strafeVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.8))
        angVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.85))

        vx = forwardVelocity * Drivetrain.maximumSpeed
        vy = strafeVelocity * Drivetrain.maximumSpeed
        omega = angVelocity * Drivetrain.maxAngularSpeed
    }
}
