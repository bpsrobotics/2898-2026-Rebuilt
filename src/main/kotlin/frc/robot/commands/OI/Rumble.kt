package frc.robot.commands.OI

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.OI

class Rumble(val side: GenericHID.RumbleType, val power: Double, val time: Double = -1.0) :
    Command() {
    val timer: Timer = Timer()

    init {
        addRequirements(OI)
    }

    override fun initialize() {
        OI.driverController.setRumble(side, power)
        timer.restart()
    }

    override fun isFinished(): Boolean {
        return time > 0 && timer.hasElapsed(time)
    }
}
