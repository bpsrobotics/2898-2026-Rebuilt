package frc.robot.commands.vision

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import kotlin.math.absoluteValue
import kotlin.math.sign

// 3, 3.2
class AlignOdometry(
    var targetPose2d: Pose2d = Pose2d(3.1, 4.24, Rotation2d(0.0)),
    val maxSpeed: Double = 0.5,
    val maxRotSpeed: Double = 1.0,
) : Command() {
    companion object {
        val yPID = PIDController(2.0, 0.3, 0.1)
        val xPID = PIDController(2.0, 0.3, 0.1)
        val rotationPID = PIDController(3.0, 0.1, 0.1)
        val xError: Double
            get() = xPID.error.absoluteValue

        val yError: Double
            get() = xPID.error.absoluteValue

        val rotationError: Double
            get() = xPID.error.absoluteValue
    }

    init {
        addRequirements(Drivetrain)
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
    }

    override fun initialize() {
        xPID.reset()
        yPID.reset()
        rotationPID.reset()

        rotationPID.setpoint = MathUtil.angleModulus(targetPose2d.rotation.radians)
        xPID.setpoint = targetPose2d.x
        yPID.setpoint = targetPose2d.y
    }

    override fun execute() {
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

        var rotationSpeed = rotationPID.calculate(Drivetrain.pose.rotation.radians)

        var xSpeed = xPID.calculate(Drivetrain.pose.x)
        var ySpeed = yPID.calculate(Drivetrain.pose.y)

        val deadzone = 0.003
        val ks = 0.05

        if (xSpeed.absoluteValue < deadzone) xSpeed = 0.0 else xSpeed += ks * xSpeed.sign
        //
        if (ySpeed.absoluteValue < deadzone) ySpeed = 0.0 else ySpeed += ks * ySpeed.sign
        //
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += ks * rotationSpeed.sign

        val totalError = rotationError + xError + yError

        if (totalError < 0.2) {
            when {
                xError > yError && xError > rotationError * 2 -> {
                    ySpeed = 0.0
                    rotationSpeed = 0.0
                }
                yPID.error > xError && yError > rotationError * 2 -> {
                    xSpeed = 0.0
                    rotationSpeed = 0.0
                }
                else -> {
                    xSpeed = 0.0
                    ySpeed = 0.0
                }
            }
        }

        Drivetrain.driveFieldOriented(
            ChassisSpeeds(
                xSpeed.clamp(-maxSpeed, maxSpeed),
                ySpeed.clamp(-maxSpeed, maxSpeed),
                rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
            )
        )
    }

    override fun isFinished(): Boolean {
        return rotationError < 0.01 && xError < 0.01 && yError < 0.01
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
