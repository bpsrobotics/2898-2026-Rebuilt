// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.engine.DashboardNumberPublisher
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI
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
    val initializeLamdbda: () -> Unit = {},
) : Command() {
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Drivetrain)
    }

    /** @suppress */
    override fun initialize() {
        initializeLamdbda()
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

object VisionTurningHandler {
    val rotationPID = PIDController(1.0, 0.0, 0.1)

    init {
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
        SmartDashboard.putData(rotationPID)
        rotationPID.setpoint = 0.0
    }

    fun initialize() {
        rotationPID.reset()
    }

    fun rotationSpeed(): Double {
        val error =
            MathUtil.inputModulus(
                Drivetrain.pose.vector2.angleTo(FieldMapREBUILTWelded.teamHub.center).asRadians -
                    Drivetrain.pose.rotation.radians,
                -PI,
                PI,
            )
        return rotationPID.calculate(-error)
    }
}
