package frc.robot.subsystems

import beaverlib.odometry.BeaverPhotonVision
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Linear.inches
import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.beaverlib.odometry.BeaverVisionCamera
import org.photonvision.PhotonPoseEstimator
import java.io.File
import kotlin.math.PI

val customField: MutableList<AprilTag> = mutableListOf(AprilTag(1, Pose3d()))

// val customVision = AprilTagFieldLayout.(mutableListOf<AprilTag>(
//    AprilTag(4, Pose3d(0.0, 0.0, 0.0, Rotation3d()
//    ))))

val Vision =
    BeaverPhotonVision(
        BeaverVisionCamera(
            "Iris_Arducam",
            Transform3d(
                -13.inches.asMeters,
                9.5.inches.asMeters,
                7.inches.asMeters,
                Rotation3d(0.0, 30.0.degrees.asRadians, PI + 20.0.degrees.asRadians),
            ),
            //            Transform3d(0.18, -0.33, 0.2, Rotation3d()),
            layout =
                AprilTagFieldLayout(
                    File(Filesystem.getDeployDirectory(), "2025-bunnybots.json").path
                ),
            strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        )
    )
