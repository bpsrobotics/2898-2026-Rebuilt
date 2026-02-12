package frc.robot.engine

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

class FFSendable(private val ff: SimpleMotorFeedforward) : Sendable {
    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("SimpleMotorFeedforward")
        builder.addDoubleProperty("kA", ff::getKa, ff::setKa)
        builder.addDoubleProperty("kS", ff::getKs, ff::setKs)
        builder.addDoubleProperty("kV", ff::getKv, ff::setKv)
    }
}
