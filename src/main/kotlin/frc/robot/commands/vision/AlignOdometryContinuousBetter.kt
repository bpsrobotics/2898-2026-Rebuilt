package frc.robot.commands.vision

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.Sugar.clamp
import frc.robot.utils.asRadians
import frc.robot.utils.degrees
import frc.robot.utils.geometry.vector2
import kotlin.math.absoluteValue
import kotlin.math.sign

private val alignOdometryContinuousTranslationPID = PIDController(2.0, 0.3, 0.1)
private val alignOdometryContinuousRotationPID = PIDController(3.0, 0.1, 0.1)
private const val alignOdometryContinuousDeadzone = 0.003
private const val alignOdometryContinuousKs = 0.05

/**
 * Auto-mode aligner that continuously drives field-oriented speeds toward the pose from
 * [targetPose2dProvider]. Built on [Drivetrain.driveLive]. Runs until interrupted.
 */
fun alignOdometryContinuousBetter(
    targetPose2dProvider: () -> Pose2d,
    maxSpeed: Double = 0.5,
    maxRotSpeed: Double = 1.0,
): Command {
    val translationPID = alignOdometryContinuousTranslationPID
    val rotationPID = alignOdometryContinuousRotationPID
    rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)

    fun computeSpeeds(): ChassisSpeeds {
        val targetPose2d = targetPose2dProvider()

        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

        var rotationSpeed =
            rotationPID.calculate(Drivetrain.pose.rotation.radians - targetPose2d.rotation.radians)

        var speed = translationPID.calculate(Drivetrain.pose.vector2.distance(targetPose2d))

        if (speed.absoluteValue < alignOdometryContinuousDeadzone) speed = 0.0
        else speed += alignOdometryContinuousKs * speed.sign
        if (rotationSpeed.absoluteValue < alignOdometryContinuousDeadzone) rotationSpeed = 0.0
        else rotationSpeed += alignOdometryContinuousKs * rotationSpeed.sign

        val totalError = rotationPID.error.absoluteValue + translationPID.error.absoluteValue

        if (totalError < 0.08) {
            if (translationPID.error.absoluteValue > rotationPID.error.absoluteValue * 2) {
                rotationSpeed = 0.0
            } else {
                speed = 0.0
            }
        }
        val travelVector =
            (Drivetrain.pose.vector2 - targetPose2d.vector2).unit * speed.clamp(-maxSpeed, maxSpeed)

        return ChassisSpeeds(
            travelVector.x.clamp(-maxSpeed, maxSpeed),
            travelVector.y.clamp(-maxSpeed, maxSpeed),
            rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
        )
    }

    return Drivetrain.driveLive(::computeSpeeds)
        .beforeStarting({
            translationPID.reset()
            rotationPID.reset()

            rotationPID.setpoint = 0.0
            translationPID.setpoint = 0.0
        })
        .finallyDo(Runnable { Drivetrain.stop() })
}
