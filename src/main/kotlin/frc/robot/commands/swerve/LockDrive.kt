package frc.robot.commands.swerve

class LockDrive : DriveManager.DriveRequestBase() {
    override var vx: Double? = 0.0
    override var vy: Double? = 0.0
    override var omega: Double? = 0.0
    override var doLock: Boolean? = false
    override val priority: Int = DriverPriority.LOCK.ordinal
}
