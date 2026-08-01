package frc.robot.utils.odometry

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult

class BeaverVisionCamera(
    val name: String,
    val robotToCamera: Transform3d,
    layout: AprilTagFieldLayout,
    strategy: PhotonPoseEstimator.PoseStrategy,
    fallbackStrategy: PhotonPoseEstimator.PoseStrategy? = null,
) {
    val cam = PhotonCamera(name)
    val results: List<PhotonPipelineResult>
        get() = cam.allUnreadResults

    val poseEstimator = PhotonPoseEstimator(layout, robotToCamera)

    var referencePose: Pose3d = Pose3d()

    init {}

    fun getMultiTagPoseWithFallback(result: PhotonPipelineResult): Pose3d? {
        if (result.targets.isEmpty()) return null
        val poseEstimation =
            if (result.multiTagResult.isPresent) poseEstimator.estimateCoprocMultiTagPose(result)
            else poseEstimator.estimateClosestToReferencePose(result, referencePose)

        if (poseEstimation.isEmpty) return null
        return poseEstimation.get().estimatedPose
    }

    fun getMultiTagPose(result: PhotonPipelineResult): Pose3d? {
        if (result.targets.isEmpty()) return null
        val poseEstimation =
            if (result.targets.size <= 1)
                poseEstimator.estimateClosestToReferencePose(result, referencePose)
            else poseEstimator.estimateCoprocMultiTagPose(result)

        if (poseEstimation.isEmpty) return null
        return poseEstimation.get().estimatedPose
    }
}
