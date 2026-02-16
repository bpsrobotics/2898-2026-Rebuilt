// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.engine.DashboardNumberPublisher
import frc.robot.subsystems.Drivetrain
import kotlin.math.pow

/**
 * A command that controls the swerve drive using joystick inputs.
 *
 * @param vForward The x velocity of the robot.
 * @param vStrafe The y velocity of the robot.
 * @param omega The angular velocity of the robot.
 * @param driveMode Boolean supplier that returns true if the robot should drive in field-oriented
 *   mode.
 * @param slowMode Boolean supplier that returns true if the robot should drive in slow mode.
 * @see Drivetrain
 */
class TeleopDriveCommand(
    val vForward: () -> Double,
    val vStrafe: () -> Double,
    val omega: () -> Double,
    val driveMode: () -> Boolean,
    val slowMode: () -> Double,
    val initializeLambda: () -> Unit = {},
) : Command() {
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Drivetrain)
    }

    /** @suppress */
    override fun initialize() {
        initializeLambda()
    }

    var forwardVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    var strafeVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    var angVelocity: Double by DashboardNumberPublisher(0.0, "Teleop/")
    var slowModeCalc: Double by DashboardNumberPublisher(0.0, "Teleop/")

    /** @suppress */
    override fun execute() {
        forwardVelocity = vForward()
        strafeVelocity = vStrafe()
        angVelocity = omega()
        slowModeCalc = slowMode()

        forwardVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.8))
        strafeVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.8))
        angVelocity *= (1.0 - (slowModeCalc.pow(3) * 0.85))

        Drivetrain.driveFieldOriented(
            ChassisSpeeds(
                forwardVelocity * Drivetrain.maximumSpeed,
                strafeVelocity * Drivetrain.maximumSpeed,
                angVelocity * Drivetrain.maxAngularSpeed,
            )
        )
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}

    /** @suppress */
    override fun isFinished(): Boolean {
        return false
    }
}
