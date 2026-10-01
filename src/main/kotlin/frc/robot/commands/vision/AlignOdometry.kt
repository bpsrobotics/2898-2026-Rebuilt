package frc.robot.commands.vision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.Sugar.clamp
import frc.robot.utils.asRadians
import frc.robot.utils.degrees
import kotlin.math.absoluteValue
import kotlin.math.sign

private val alignOdometryYPID = PIDController(2.0, 0.3, 0.1)
private val alignOdometryXPID = PIDController(2.0, 0.3, 0.1)
private val alignOdometryRotationPID = PIDController(3.0, 0.1, 0.1)

// 3, 3.2
/**
 * Auto-mode aligner that drives field-oriented speeds to [targetPose2d]. Built on
 * [Drivetrain.driveLive].
 */
fun alignOdometry(
    targetPose2d: Pose2d = Pose2d(3.1, 4.24, Rotation2d(0.0)),
    maxSpeed: Double = 0.5,
    maxRotSpeed: Double = 1.0,
): Command {
    val xPID = alignOdometryXPID
    val yPID = alignOdometryYPID
    val rotationPID = alignOdometryRotationPID
    rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)

    fun computeSpeeds(): ChassisSpeeds {
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

        var rotationSpeed = rotationPID.calculate(Drivetrain.pose.rotation.radians)

        var xSpeed = xPID.calculate(Drivetrain.pose.x)
        var ySpeed = yPID.calculate(Drivetrain.pose.y)

        val deadzone = 0.003
        val ks = 0.05

        if (xSpeed.absoluteValue < deadzone) xSpeed = 0.0 else xSpeed += ks * xSpeed.sign
        if (ySpeed.absoluteValue < deadzone) ySpeed = 0.0 else ySpeed += ks * ySpeed.sign
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += ks * rotationSpeed.sign

        val totalError =
            rotationPID.error.absoluteValue + xPID.error.absoluteValue + yPID.error.absoluteValue

        if (totalError < 0.2) {
            when {
                xPID.error.absoluteValue > yPID.error.absoluteValue &&
                    xPID.error.absoluteValue > rotationPID.error.absoluteValue * 2 -> {
                    ySpeed = 0.0
                    rotationSpeed = 0.0
                }
                yPID.error.absoluteValue > xPID.error.absoluteValue &&
                    yPID.error.absoluteValue > rotationPID.error.absoluteValue * 2 -> {
                    xSpeed = 0.0
                    rotationSpeed = 0.0
                }
                else -> {
                    xSpeed = 0.0
                    ySpeed = 0.0
                }
            }
        }

        return ChassisSpeeds(
            xSpeed.clamp(-maxSpeed, maxSpeed),
            ySpeed.clamp(-maxSpeed, maxSpeed),
            rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
        )
    }

    return Drivetrain.driveLive(::computeSpeeds)
        .beforeStarting({
            xPID.reset()
            yPID.reset()
            rotationPID.reset()

            rotationPID.setpoint = MathUtil.angleModulus(targetPose2d.rotation.radians)
            xPID.setpoint = targetPose2d.x
            yPID.setpoint = targetPose2d.y
        })
        .until {
            rotationPID.error.absoluteValue < 0.01 &&
                xPID.error.absoluteValue < 0.01 &&
                yPID.error.absoluteValue < 0.01
        }
        .finallyDo(Runnable { Drivetrain.stop() })
}
