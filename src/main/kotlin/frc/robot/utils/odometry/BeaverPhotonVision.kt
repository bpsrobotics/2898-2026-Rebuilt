package frc.robot.utils.odometry

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.misc.BiSignal
import org.photonvision.targeting.PhotonPipelineResult

class BeaverPhotonVision(vararg val cameras: BeaverVisionCamera) : SubsystemBase() {
    val listeners = BiSignal<PhotonPipelineResult, BeaverVisionCamera>()

    override fun periodic() {
        for (camera in cameras) {
            for (result in camera.results) {
                listeners.update(result, camera)
            }
        }
    }

    fun setAllCameraReferences(pose: Pose3d) {
        for (camera in cameras) {
            camera.referencePose = pose
        }
    }

    fun getStandardDev(STDVX: Double, STDVY: Double, rotationSTD: Double): Matrix<N3, N1> {
        val stdv = Matrix(Nat.N3(), Nat.N1())
        stdv.set(0, 0, STDVX)
        stdv.set(1, 0, STDVY)
        stdv.set(2, 0, rotationSTD)
        return stdv
    }
}
