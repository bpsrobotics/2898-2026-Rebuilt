package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake

class RunIntake(private val power: Double) : Command() {
    init {
        addRequirements(Intake)
    }

    override fun initialize() {
        Intake.runAtPower(power)
    }

    override fun end(interrupted: Boolean) {
        Intake.stop()
    }
}
