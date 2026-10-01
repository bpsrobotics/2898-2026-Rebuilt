package frc.robot.commands.vision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.Sugar.clamp
import frc.robot.utils.asMeters
import frc.robot.utils.asRadians
import frc.robot.utils.convert
import frc.robot.utils.degrees
import frc.robot.utils.geometry.Vector2
import frc.robot.utils.geometry.vector2
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

private val circleAlignCirclePID = PIDController(4.0, 0.3, 0.1)
private val circleAlignDistancePID = PIDController(2.0, 0.3, 0.1)
private val circleAlignRotationPID = PIDController(3.0, 0.1, 0.1)

/**
 * Auto-mode aligner that orbits [targetCenter] at [desiredDistance] while holding the heading from
 * [angleProvider]. Controls all axes field-oriented via [Drivetrain.driveLive]. Runs until
 * interrupted.
 */
fun circleAlign(
    targetCenter: () -> Vector2,
    angleProvider: () -> Angle,
    desiredDistance: () -> Distance,
    maxSpeed: Double = 5.0,
    maxRotSpeed: Double = 1.0,
    initializeLambda: () -> Unit = {},
    endLambda: () -> Unit = {},
): Command {
    val circlePID = circleAlignCirclePID
    val distancePID = circleAlignDistancePID
    val rotationPID = circleAlignRotationPID
    rotationPID.enableContinuousInput(
        (-180).degrees.convert(Units.Radians),
        180.degrees.convert(Units.Radians),
    )

    fun computeSpeeds(): ChassisSpeeds {
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()
        val currentAngleToCenter = Drivetrain.pose.vector2.angleTo(targetCenter())

        var rotationSpeed =
            rotationPID.calculate(Drivetrain.pose.rotation.radians - angleProvider().asRadians)

        var circleSpeed =
            circlePID.calculate(
                MathUtil.angleModulus(
                    Drivetrain.pose.vector2.angleTo(targetCenter()).asRadians -
                        angleProvider().asRadians
                )
            )
        var distanceSpeed =
            distancePID.calculate(
                Drivetrain.pose.vector2.distance(targetCenter()) - desiredDistance().asMeters
            )
        println(Drivetrain.pose.vector2.distance(targetCenter()) - desiredDistance().asMeters)

        val deadzone = 0.01
        val ks = 0.05

        if (circleSpeed.absoluteValue < 0.05) circleSpeed = 0.0
        else circleSpeed += ks * circleSpeed.sign
        if (distanceSpeed.absoluteValue < 0.1) distanceSpeed = 0.0
        else distanceSpeed += ks * distanceSpeed.sign
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += ks * rotationSpeed.sign

        val currentAngleRadians = currentAngleToCenter.convert(Units.Radians)
        val xSpeed =
            circleSpeed * -sin(currentAngleRadians) + cos(currentAngleRadians) * distanceSpeed
        val ySpeed =
            circleSpeed * cos(currentAngleRadians) + sin(currentAngleRadians) * distanceSpeed

        return ChassisSpeeds(
            xSpeed.clamp(-maxSpeed, maxSpeed),
            ySpeed.clamp(-maxSpeed, maxSpeed),
            rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
        )
    }

    return Drivetrain.driveLive(::computeSpeeds)
        .beforeStarting({
            circlePID.reset()
            distancePID.reset()
            rotationPID.reset()

            rotationPID.setpoint = 0.0
            circlePID.setpoint = 0.0
            distancePID.setpoint = 0.0
            initializeLambda()
        })
        .finallyDo(
            Runnable {
                Drivetrain.stop()
                endLambda()
            }
        )
}
