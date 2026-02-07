package frc.robot.commands.vision

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import kotlin.math.absoluteValue
import kotlin.math.sign

class CircleAlign(
    var targetCenter: () -> Vector2,
    val angleProvider: () -> AngleUnit,
    val desiredDistance: () -> DistanceUnit,
    val maxSpeed: Double = 5.0,
    val maxRotSpeed: Double = 1.0,
    val initializeLambda: () -> Unit = {},
    val endLambda: () -> Unit = {},
) : Command() {
    val timer = Timer()

    companion object {
        val circlePID = PIDController(4.0, 0.3, 0.1)
        val distancePID = PIDController(2.0, 0.3, 0.1)
        val rotationPID = PIDController(3.0, 0.1, 0.1)
        val circleError: Double
            get() = circlePID.error.absoluteValue

        val distanceError: Double
            get() = distancePID.error.absoluteValue

        val rotationError: Double
            get() = rotationPID.error.absoluteValue
    }

    init {
        addRequirements(Drivetrain)
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
    }

    override fun initialize() {
        circlePID.reset()
        distancePID.reset()
        rotationPID.reset()

        rotationPID.setpoint = 0.0
        circlePID.setpoint = 0.0
        distancePID.setpoint = 0.0
        initializeLambda()
    }

    override fun execute() {

        // if (rotationError < 0.01 && xError < 0.01 && yError < 0.01) return

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
        //
        if (distanceSpeed.absoluteValue < 0.1) distanceSpeed = 0.0
        else distanceSpeed += ks * distanceSpeed.sign
        //
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += ks * rotationSpeed.sign

        val xSpeed =
            circleSpeed * -currentAngleToCenter.sin() + currentAngleToCenter.cos() * distanceSpeed
        val ySpeed =
            circleSpeed * currentAngleToCenter.cos() + currentAngleToCenter.sin() * distanceSpeed

        Drivetrain.driveFieldOriented(
            ChassisSpeeds(
                xSpeed.clamp(-maxSpeed, maxSpeed),
                ySpeed.clamp(-maxSpeed, maxSpeed),
                rotationSpeed.clamp(-maxRotSpeed, maxRotSpeed),
            )
        )
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
        endLambda()
    }
}
