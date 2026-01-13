package frc.robot.commands.vision

import beaverlib.utils.Units.Angular.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.sign

class FollowApriltag(val apriltagId: Int) : Command() {
    init {
        addRequirements(Drivetrain)
    }

    var desiredTag: PhotonTrackedTarget? = null

    override fun initialize() {
        Vision.listeners.add(
            "FollowTag",
            { result, camera ->
                val desiredTagA = result.targets.filter { it.fiducialId == apriltagId }
                if (desiredTagA.isEmpty()) {
                    desiredTag = null
                    return@add
                }
                desiredTag = desiredTagA.first()
            },
        )
    }

    override fun execute() {
        if (desiredTag == null) {
            Drivetrain.driveRobotOriented(ChassisSpeeds())
            return
        }
        val yawToTag = desiredTag!!.bestCameraToTarget.rotation.z
        val kp = -3.0
        val error = Rotation2d.fromDegrees(180.0).minus(Rotation2d.fromRadians(yawToTag)).radians
        println(error)

        if (error > 3.degrees.asRadians) {
            Drivetrain.driveRobotOriented(
                ChassisSpeeds(0.0, 0.0, kp * error + (-0.01 * error.sign))
            )
            return
        }

        val distanceToTag = desiredTag!!.bestCameraToTarget.x
        if (distanceToTag > 1.1) Drivetrain.driveRobotOriented(ChassisSpeeds(1.0, 0.0, 0.0))
        if (distanceToTag < 0.9) Drivetrain.driveRobotOriented(ChassisSpeeds(-1.0, 0.0, 0.0))
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("FollowTag")
    }
}
