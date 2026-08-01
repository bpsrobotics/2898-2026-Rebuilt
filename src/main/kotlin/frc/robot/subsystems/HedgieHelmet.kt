package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.utils.fieldmap.FieldMapREBUILTWelded
import frc.robot.utils.meters

object HedgieHelmet {
    val trenchDriveTrigger = Trigger { willCollideWithTrench() && !Vision.cameras.isEmpty() }

    val PADDING = 2.meters

    private fun willCollideWithTrench(): Boolean {
        val robotX = Drivetrain.pose.x.meters

        for (x in
            arrayOf(
                FieldMapREBUILTWelded.RedAllianceAreaLineX,
                FieldMapREBUILTWelded.BlueAllianceAreaLineX,
            )) {
            if (x - PADDING <= robotX && robotX <= x + PADDING) return true
        }

        return false
    }
}
