package frc.robot.commands.vision

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.Sugar.clamp
import frc.robot.utils.asRadians
import frc.robot.utils.degrees
import frc.robot.utils.geometry.Vector2
import frc.robot.utils.radians
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import org.photonvision.targeting.PhotonTrackedTarget

/**
 * Auto-mode aligner that drives robot-oriented speeds to hold [xToTag]/[yToTag] offsets from an
 * AprilTag. Built on [Drivetrain.driveLiveRobotOriented].
 */
fun followApriltagGood(apriltagId: Int, yToTag: Double = 0.0, xToTag: Double = 1.0): Command {
    val timer = Timer()
    val xPID = PIDController(2.0, 0.1, 0.1)
    val yPID = PIDController(2.0, 0.1, 0.1)
    val rotationPID = PIDController(2.0, 0.2, 0.0)
    rotationPID.enableContinuousInput(-180.degrees.asRadians, 180.degrees.asRadians)

    var subscription: Vision.Subscription? = null
    var desiredTag: PhotonTrackedTarget? = null
    var timeSinceTagSeen = 0.0
    var trueFrames = 0
    var lastDesiredTag: PhotonTrackedTarget? = null

    fun computeSpeeds(): ChassisSpeeds {
        val tag = desiredTag
        if (tag == null || timer.get() - timeSinceTagSeen > 0.1) {
            return ChassisSpeeds()
        }
        val yawToTag = tag.bestCameraToTarget.rotation.z.radians
        val xDistVector = tag.bestCameraToTarget.x
        val yDistVector = tag.bestCameraToTarget.y

        val robotPos = Vector2(0.0, -Vision.cameras.first().robotToCamera.y)
        val xVector = Vector2.new(yawToTag, xDistVector)
        val yVector = Vector2.new(yawToTag - 90.degrees, yDistVector)

        val tagPos = robotPos + xVector + yVector

        val xSpeed = yPID.calculate(tagPos.y)
        val ySpeed = xPID.calculate(tagPos.x)
        SmartDashboard.putNumber("xSpeed", xSpeed)
        println("xSpeed $xSpeed, ySpeed $ySpeed")

        var yDriveSpeed = ySpeed * cos(-yawToTag.asRadians)
        var xDriveSpeed = ySpeed * sin(-yawToTag.asRadians)

        var rotationSpeed = -rotationPID.calculate(yawToTag.asRadians)

        val deadzone = 0.02
        if (xDriveSpeed.absoluteValue < deadzone) xDriveSpeed = 0.0
        else xDriveSpeed += deadzone * xSpeed.sign
        if (yDriveSpeed.absoluteValue < deadzone) yDriveSpeed = 0.0
        else yDriveSpeed += deadzone * ySpeed.sign
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += deadzone * rotationSpeed.sign

        return ChassisSpeeds(
            xDriveSpeed.clamp(-0.5, 0.5),
            yDriveSpeed.clamp(-0.5, 0.5),
            rotationSpeed.clamp(-0.5, 0.5),
        )
    }

    return Drivetrain.driveLiveRobotOriented(::computeSpeeds)
        .beforeStarting({
            subscription = Vision.onResult { result, _ ->
                val desiredTagA = result.targets.filter { it.fiducialId == apriltagId }
                if (desiredTagA.isEmpty() || desiredTagA.first().poseAmbiguity > 0.5) {
                    return@onResult
                }
                timeSinceTagSeen = timer.get()
                desiredTag = desiredTagA.first()
            }
            timer.restart()
            yPID.setpoint = yToTag
            xPID.setpoint = xToTag
            rotationPID.setpoint = 180.degrees.asRadians
            yPID.reset()
            rotationPID.reset()
            desiredTag = null
            trueFrames = 0
        })
        .until {
            val aligned =
                desiredTag != null &&
                    yPID.error < 0.02 &&
                    rotationPID.error < 0.02 &&
                    timer.hasElapsed(2.0) &&
                    desiredTag != lastDesiredTag
            if (aligned) trueFrames += 1 else trueFrames = 0
            lastDesiredTag = desiredTag
            trueFrames > 3
        }
        .finallyDo(
            Runnable {
                subscription?.close()
                Drivetrain.stop()
            }
        )
}
