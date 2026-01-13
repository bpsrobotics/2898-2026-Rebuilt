package frc.robot.commands.OI

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain

class NavXReset : Command() {
    private val time = Timer()
    private val swerve: Drivetrain

    init {
        addRequirements(Drivetrain)
        this.swerve = Drivetrain
    }

    override fun initialize() {
        time.reset()
        time.start()
    }

    override fun isFinished(): Boolean {
        return time.hasElapsed(0.15)
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            Rumble(GenericHID.RumbleType.kRightRumble, 0.25, 0.2).schedule()
            swerve.zeroGyro()
        }
    }
}
