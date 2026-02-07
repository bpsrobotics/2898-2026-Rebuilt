// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
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

    fun desiredRotation(): AngleUnit {
        val nextFramePos = (Drivetrain.pose.vector2 + Drivetrain.fieldVelocity.vector2 * 0.02)
        SmartDashboard.putString(
            "CurrentFieldPos",
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)).toString(),
        )
        println(FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)))
        if (
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)) ==
                FieldMapREBUILTWelded.getTeamAllianceArea()
        ) {
            return nextFramePos.angleTo(FieldMapREBUILTWelded.teamHub.center)
        }
        if (
            FieldMapREBUILTWelded.getPoseAllianceArea(nextFramePos.toPose2d(0.0.radians)) ==
                FieldMapREBUILTWelded.AllianceArea.Center
        ) {
            if (
                nextFramePos.y + Drivetrain.Constants.RobotWidth.asMeters / 2 <
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight.y ||
                    nextFramePos.y - Drivetrain.Constants.RobotWidth.asMeters / 2 >
                        FieldMapREBUILTWelded.teamHub.shape.topRight.y
            ) {
                return Vector2(nextFramePos.x, 0.0)
                    .angleTo(Vector2(FieldMapREBUILTWelded.teamHub.shape.bottomRight.x, 0.0))
            }
            if (nextFramePos.y < FieldMapREBUILTWelded.teamHub.center.y) {
                return nextFramePos.angleTo(
                    FieldMapREBUILTWelded.teamHub.shape.bottomRight - Vector2(0.0, 0.1)
                )
            }
            return nextFramePos.angleTo(
                FieldMapREBUILTWelded.teamHub.shape.topRight + Vector2(0.0, 0.1)
            )
        }
        return Drivetrain.pose.rotation.radians.radians
    }

    fun rotationSpeed(): Double {
        SmartDashboard.putNumber(
            "Odometry/DesiredRotation",
            MathUtil.inputModulus(desiredRotation().asRadians, -PI, PI),
        )
        val error =
            MathUtil.inputModulus(
                desiredRotation().asRadians - Drivetrain.pose.rotation.radians,
                -PI,
                PI,
            )
        return rotationPID.calculate(-error)
    }
}
