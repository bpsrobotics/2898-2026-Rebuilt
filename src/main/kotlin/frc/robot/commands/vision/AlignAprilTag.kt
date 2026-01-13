package frc.robot.commands

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import org.photonvision.targeting.PhotonTrackedTarget

class AlignAprilTag(val apriltagId: Int, var yToTag: Double = 0.0) : Command() {
    val timer = Timer()
    val yPID = PIDController(2.0, 0.1, 0.1)
    val rotationPID = PIDController(2.0, 0.2, 0.0)

    init {
        addRequirements(Drivetrain)
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
    }

    var startPose = Pose2d()
    var desiredTag: PhotonTrackedTarget? = null
    var distX = 0.0
    var timeSinceTagSeen = 0.0

    override fun initialize() {
        Vision.listeners.add(
            "AlignTag",
            { result, camera ->
                val desiredTagA = result.targets.filter { it.fiducialId == apriltagId }
                if (desiredTagA.isEmpty() || desiredTagA.first().poseAmbiguity > 0.5) {
                    return@add
                }
                timeSinceTagSeen = timer.get()
                desiredTag = desiredTagA.first()
                // val robotToTag = camera.poseEstimator.update(result)
            },
        )
        timer.restart()
        yPID.setpoint = yToTag
        rotationPID.setpoint = 180.degrees.asRadians
        yPID.reset()
        rotationPID.reset()
        desiredTag = null
        trueFrames = 0
    }

    var estimatedTagPos = Vector2.new(0, 0)

    override fun execute() {

        // println("${yPID.atSetpoint()} && ${rotationPID.atSetpoint()}")
        // println(desiredTag)

        if (desiredTag == null || timer.get() - timeSinceTagSeen > 0.1) {
            Drivetrain.stop()
            return
        }
        val yawToTag = desiredTag!!.bestCameraToTarget.rotation.z.radians
        val xDistVector = desiredTag!!.bestCameraToTarget.x
        val yDistVector = desiredTag!!.bestCameraToTarget.y
        // println(error)
        val robotPos = Vector2(0.0, -Vision.cameras.first().robotToCamera.y)
        val xVector = Vector2.new(yawToTag, xDistVector)
        val yVector = Vector2.new(yawToTag - 90.degrees, yDistVector)

        val tagPos = robotPos + xVector + yVector
        // println("$tagPos, rotation:${yawToTag.asDegrees}")

        val speed = yPID.calculate(tagPos.y).clamp(-1.0, 1.0)
        var ySpeed = speed * cos(-yawToTag.asRadians).clamp(-1.0, 1.0)
        var xSpeed = speed * sin(-yawToTag.asRadians).clamp(-1.0, 1.0)

        var rotationSpeed = -rotationPID.calculate(yawToTag.asRadians).clamp(-1.0, 1.0)

        val deadzone = 0.02
        if (xSpeed.absoluteValue < deadzone) xSpeed = 0.0 else xSpeed += deadzone * xSpeed.sign
        if (ySpeed.absoluteValue < deadzone) ySpeed = 0.0 else ySpeed += deadzone * ySpeed.sign
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += deadzone * rotationSpeed.sign
        // println("rotationSpeed$rotationSpeed")

        Drivetrain.driveRobotOriented(ChassisSpeeds(xSpeed, ySpeed, rotationSpeed))
    }

    var trueFrames = 0
    var lastDesiredTag: PhotonTrackedTarget? = null

    override fun isFinished(): Boolean {

        if (
            desiredTag != null &&
                yPID.error < 0.02 &&
                rotationPID.error < 0.02 &&
                timer.hasElapsed(2.0) &&
                desiredTag != lastDesiredTag
        )
            trueFrames += 1
        else trueFrames = 0
        lastDesiredTag = desiredTag
        return trueFrames > 3
    }

    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("AlignTag")
        Drivetrain.stop()
    }
}
