package frc.robot.commands.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.engine.DashboardString
import frc.robot.subsystems.Drivetrain

class DriveManager : Command() {
    interface DriveRequest {
        val vx: Double?
        val vy: Double?
        val omega: Double?
        val doLock: Boolean?
        val isEnabled: Boolean
        val priority: Int
    }

    abstract class DriveRequestBase : Command(), DriveRequest {
        override var vx: Double? = null
        override var vy: Double? = null
        override var omega: Double? = null
        override var doLock: Boolean? = null
        override var isEnabled: Boolean = false

        override fun initialize() {
            isEnabled = true
        }

        override fun end(interrupted: Boolean) {
            isEnabled = false
        }
    }

    init {
        addRequirements(Drivetrain)
    }

    var requestsDebug by DashboardString("", "DriveManager")
    var requestsEnabledDebug by DashboardString("", "DriveManager")

    private val requests = mutableListOf<DriveRequest>()

    fun <T : DriveRequest> defineDriver(rq: T): T {
        requests.add(rq)
        return rq
    }

    override fun execute() {
        requestsDebug = requests.map { it.priority }.joinToString()
        requestsEnabledDebug = requests.filter { it.isEnabled }.map { it.priority }.joinToString()

        val doLock =
            requests
                .filter { it.doLock != null && it.isEnabled }
                .maxByOrNull { it.priority }
                ?.doLock ?: false
        if (doLock) {
            Drivetrain.lock()
            return
        }

        val vx =
            requests.filter { it.vx != null && it.isEnabled }.maxByOrNull { it.priority }?.vx ?: 0.0
        val vy =
            requests.filter { it.vy != null && it.isEnabled }.maxByOrNull { it.priority }?.vy ?: 0.0
        val omega =
            requests.filter { it.omega != null && it.isEnabled }.maxByOrNull { it.priority }?.omega
                ?: 0.0
        val speeds = ChassisSpeeds(vx, vy, omega)

        Drivetrain.driveFieldOriented(speeds)
    }
}
