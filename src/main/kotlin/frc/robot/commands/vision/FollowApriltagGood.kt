package frc.robot.commands

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import org.photonvision.targeting.PhotonTrackedTarget

class FollowApriltagGood(val apriltagId: Int, var yToTag: Double = 0.0, var xToTag: Double = 1.0) :
    Command() {
    val timer = Timer()
    val xPID = PIDController(2.0, 0.1, 0.1)
    val yPID = PIDController(2.0, 0.1, 0.1)
    val rotationPID = PIDController(2.0, 0.2, 0.0)

    init {
        addRequirements(Drivetrain)
        rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)
    }

    var desiredTag: PhotonTrackedTarget? = null
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
        xPID.setpoint = xToTag
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

        val robotPos = Vector2(0.0, -Vision.cameras.first().robotToCamera.y)
        val xVector = Vector2.new(yawToTag, xDistVector)
        val yVector = Vector2.new(yawToTag - 90.degrees, yDistVector)

        val tagPos = robotPos + xVector + yVector

        val xSpeed = yPID.calculate(tagPos.y)
        val ySpeed = xPID.calculate(tagPos.x)
        SmartDashboard.putNumber("xSpeed", xSpeed)
        // println("xPos ${tagPos.x}, yPos ${tagPos.y}")
        println("xSpeed $xSpeed, ySpeed $ySpeed")

        var yDriveSpeed = ySpeed * cos(-yawToTag.asRadians) // + xSpeed * sin(-yawToTag.asRadians)
        var xDriveSpeed = ySpeed * sin(-yawToTag.asRadians) // + xSpeed * cos(-yawToTag.asRadians)

        var rotationSpeed = -rotationPID.calculate(yawToTag.asRadians)

        val deadzone = 0.02
        if (xDriveSpeed.absoluteValue < deadzone) xDriveSpeed = 0.0
        else xDriveSpeed += deadzone * xSpeed.sign
        if (yDriveSpeed.absoluteValue < deadzone) yDriveSpeed = 0.0
        else yDriveSpeed += deadzone * ySpeed.sign
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += deadzone * rotationSpeed.sign

        Drivetrain.driveRobotOriented(
            ChassisSpeeds(
                xDriveSpeed.clamp(-0.5, 0.5),
                yDriveSpeed.clamp(-0.5, 0.5),
                rotationSpeed.clamp(-0.5, 0.5),
            )
        )
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
