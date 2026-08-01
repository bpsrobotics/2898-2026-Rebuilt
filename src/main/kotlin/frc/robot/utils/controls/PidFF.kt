package frc.robot.utils.controls

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

class PidFF(pidConstants: PIDConstants, ffConstants: SimpleMotorFeedForwardConstants) : Sendable {
    val pid = PIDController(pidConstants.p, pidConstants.i, pidConstants.d)
    val feedforward = SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA)

    var setpoint: Double
        get() = pid.setpoint
        set(value) {
            pid.setpoint = value
        }

    fun calculate(measurement: Double): Double {
        return pid.calculate(measurement) + feedforward.calculate(setpoint)
    }

    fun atSetpoint(): Boolean {
        return pid.atSetpoint()
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("kS", feedforward::getKs, feedforward::setKs)
        builder.addDoubleProperty("kV", feedforward::getKv, feedforward::setKv)
        builder.addDoubleProperty("kA", feedforward::getKa, feedforward::setKa)
        builder.addDoubleProperty("kP", pid::getP, pid::setP)
        builder.addDoubleProperty("kI", pid::getI, pid::setI)
        builder.addDoubleProperty("kD", pid::getD, pid::setD)
        builder.addDoubleProperty("setpoint", { setpoint }, { setpoint = it })
    }
}
