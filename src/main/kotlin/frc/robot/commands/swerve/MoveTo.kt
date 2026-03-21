package frc.robot.commands.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI

class MoveTo(val targetPose: () -> Pose2d) : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.MOVE_TO.ordinal
    private val xPID = PIDController(2.0, 0.3, 0.1)
    private val yPID = PIDController(2.0, 0.3, 0.1)
    private val rotationPID = PIDController(2.0, 0.01, 0.2)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
        xPID.setTolerance(0.1)
        yPID.setTolerance(0.1)
        rotationPID.setTolerance(0.2)
    }

    override fun execute() {
        val target = targetPose()
        xPID.setpoint = target.x
        yPID.setpoint = target.y
        rotationPID.setpoint = target.rotation.radians
        val pose = Drivetrain.pose
        vx = xPID.calculate(pose.x)
        vy = yPID.calculate(pose.y)
        omega = rotationPID.calculate(pose.rotation.radians)
    }

    override fun isFinished(): Boolean {
        return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint()
    }
}
