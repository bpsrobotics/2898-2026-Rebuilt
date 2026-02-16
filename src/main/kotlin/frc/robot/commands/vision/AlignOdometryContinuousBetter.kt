package frc.robot.commands.vision

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import kotlin.math.absoluteValue
import kotlin.math.sign

class AlignOdometryContinuousBetter(
    var targetPose2dProvider: () -> Pose2d,
    val maxSpeed: Double = 0.5,
    val maxRotSpeed: Double = 1.0,
) : Command() {
    companion object {
        val translationPID = PIDController(2.0, 0.3, 0.1)
        val rotationPID = PIDController(3.0, 0.1, 0.1)
        const val deadzone = 0.003
        const val ks = 0.05

        val translationError: Double
            get() = translationPID.error.absoluteValue

        val rotationError: Double
            get() = rotationPID.error.absoluteValue
    }

    init {
        addRequirements(Drivetrain)
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
    }

    override fun initialize() {
        translationPID.reset()
        rotationPID.reset()

        rotationPID.setpoint = 0.0
        translationPID.setpoint = 0.0
    }

    override fun execute() {
        val targetPose2d = targetPose2dProvider()

        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

        var rotationSpeed =
            rotationPID.calculate(Drivetrain.pose.rotation.radians - targetPose2d.rotation.radians)

        var speed = translationPID.calculate(Drivetrain.pose.vector2.distance(targetPose2d))

        if (speed.absoluteValue < deadzone) speed = 0.0 else speed += ks * speed.sign
        //
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += ks * rotationSpeed.sign

        val totalError = rotationError + translationError

        if (totalError < 0.08) {
            if (translationError > rotationError * 2) {
                rotationSpeed = 0.0
            } else {
                speed = 0.0
            }
        }
        val travelVector =
            (Drivetrain.pose.vector2 - targetPose2d.vector2).unit * speed.clamp(-maxSpeed, maxSpeed)

        Drivetrain.driveFieldOriented(
            ChassisSpeeds(
                travelVector.x.clamp(-maxSpeed, maxSpeed),
                travelVector.y.clamp(-maxSpeed, maxSpeed),
                rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
            )
        )
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
