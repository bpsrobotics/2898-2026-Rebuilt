package frc.robot.engine

import beaverlib.controls.PIDConstants
import beaverlib.controls.toPID
import beaverlib.utils.Units.Angular.AngleUnit
import kotlin.math.PI

class PIDSinController(constants: PIDConstants, val kSin: Double) {
    val PID = constants.toPID()

    init {
        PID.enableContinuousInput(-PI, PI)
    }

    var setpoint
        get() = PID.setpoint
        set(value) {
            PID.setpoint = value
        }

    fun calculate(measurement: AngleUnit): Double {
        return PID.calculate(measurement.asRadians) + measurement.sin() * kSin
    }
}

fun PIDConstants.toPIDSin(kSin: Double) = PIDSinController(this, kSin)
