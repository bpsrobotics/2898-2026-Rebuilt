package frc.robot.subsystems

import beaverlib.odometry.BeaverPhotonVision
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Linear.inches
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.beaverlib.odometry.BeaverVisionCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.math.PI

val Vision =
    BeaverPhotonVision(
        BeaverVisionCamera(
            "Iris_Arducam",
            Transform3d(
                -13.inches.asMeters, // -13
                -9.5.inches.asMeters,
                7.inches.asMeters,
                Rotation3d(0.0, 30.0.degrees.asRadians, PI + 7.0.degrees.asRadians),
            ),
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
            strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        )
    )
