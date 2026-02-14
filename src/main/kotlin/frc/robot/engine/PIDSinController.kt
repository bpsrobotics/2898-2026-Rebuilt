package frc.robot.engine

import beaverlib.controls.PIDConstants
import beaverlib.controls.toPID
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.PI

class PIDSinController(constants: PIDConstants, var kSin: Double) : Sendable {
    val pid = constants.toPID()

    init {
        pid.enableContinuousInput(-PI, PI)
    }

    var setpoint: AngleUnit
        get() = pid.setpoint.radians
        set(value) {
            pid.setpoint = value.asRadians
        }

    fun calculate(measurement: AngleUnit): Double {
        return pid.calculate(measurement.asRadians) + measurement.sin() * kSin
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("PIDSinController")
        builder.addDoubleProperty("kP", pid::getP, pid::setP)
        builder.addDoubleProperty("kI", pid::getI, pid::setI)
        builder.addDoubleProperty("kD", pid::getD, pid::setD)
        builder.addDoubleProperty("kSin", { kSin }, { value -> kSin = value })
        builder.addDoubleProperty(
            "setpoint",
            { setpoint.asRadians },
            { value -> setpoint = value.radians },
        )
    }
}

fun PIDConstants.toPIDSin(kSin: Double) = PIDSinController(this, kSin)
