package frc.robot.commands.swerve

import edu.wpi.first.math.controller.PIDController
import frc.robot.subsystems.Drivetrain

class TrenchAlign : DriveManager.DriveRequestBase() {
    override val priority: Int = DriverPriority.TRENCH_ALIGN.ordinal

    private object Constants {
        const val TOP_LINE = 7.426833
        const val BOTTOM_LINE = 0.639445
        const val HALFWAY = (TOP_LINE + BOTTOM_LINE) / 2.0
    }

    private val pid = PIDController(2.0, 0.3, 0.1)

    override fun execute() {
        pid.setpoint =
            if (Drivetrain.pose.y > Constants.HALFWAY) Constants.TOP_LINE else Constants.BOTTOM_LINE
        vy = pid.calculate(Drivetrain.pose.y)
    }
}
