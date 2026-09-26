package frc.robot.commands.vision

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.asRadians
import frc.robot.utils.degrees
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.sign

/**
 * Auto-mode follower that rotates toward an AprilTag then drives to 1m distance, all
 * robot-oriented. Built on [Drivetrain.driveLiveRobotOriented]. Runs until interrupted.
 */
fun followApriltag(apriltagId: Int): Command {
    var subscription: Vision.Subscription? = null
    var desiredTag: PhotonTrackedTarget? = null

    fun computeSpeeds(): ChassisSpeeds {
        val tag = desiredTag ?: return ChassisSpeeds()
        val yawToTag = tag.bestCameraToTarget.rotation.z
        val kp = -3.0
        val error = Rotation2d.fromDegrees(180.0).minus(Rotation2d.fromRadians(yawToTag)).radians
        println(error)

        if (error > 3.degrees.asRadians) {
            return ChassisSpeeds(0.0, 0.0, kp * error + (-0.01 * error.sign))
        }

        val distanceToTag = tag.bestCameraToTarget.x
        if (distanceToTag > 1.1) return ChassisSpeeds(1.0, 0.0, 0.0)
        if (distanceToTag < 0.9) return ChassisSpeeds(-1.0, 0.0, 0.0)
        return ChassisSpeeds()
    }

    return Drivetrain.driveLiveRobotOriented(::computeSpeeds)
        .beforeStarting(
            Runnable {
                subscription = Vision.onResult { result, _ ->
                    val desiredTagA = result.targets.filter { it.fiducialId == apriltagId }
                    if (desiredTagA.isEmpty()) {
                        desiredTag = null
                        return@onResult
                    }
                    desiredTag = desiredTagA.first()
                }
            }
        )
        .finallyDo(Runnable { subscription?.close() })
}
