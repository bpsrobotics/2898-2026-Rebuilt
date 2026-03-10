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

@Suppress("Typo")
val Vision =
    BeaverPhotonVision(
        BeaverVisionCamera(
            "Iris_Arducam",
            Transform3d(
                -1.021.inches.asMeters, // -13
                5.276.inches.asMeters,
                19.010.inches.asMeters,
                Rotation3d(0.0, -30.0.degrees.asRadians, 30.degrees.asRadians),
            ),
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
            strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        ),
        BeaverVisionCamera(
            "Retina_Arducam",
            Transform3d(
                -1.021.inches.asMeters, // -13
                -5.276.inches.asMeters,
                19.010.inches.asMeters,
                Rotation3d(0.0, -30.0.degrees.asRadians, -30.degrees.asRadians),
            ),
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
            strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        ),
    )
