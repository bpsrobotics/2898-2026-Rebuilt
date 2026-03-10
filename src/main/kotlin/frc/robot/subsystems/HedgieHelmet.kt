package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Linear.meters
import edu.wpi.first.wpilibj2.command.button.Trigger

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
