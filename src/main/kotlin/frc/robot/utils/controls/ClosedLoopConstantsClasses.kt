package frc.robot.utils.controls

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

data class SimpleMotorFeedForwardConstants(val kS: Double, val kV: Double, val kA: Double)

data class ArmFeedForwardConstants(
    val kS: Double,
    val kG: Double,
    val kV: Double,
    val kA: Double = 0.0,
)

data class PIDConstants(val p: Double, val i: Double, val d: Double)

fun SimpleMotorFeedForwardConstants.toFeedForward(): SimpleMotorFeedforward {
    return SimpleMotorFeedforward(this.kS, this.kV, this.kA)
}

fun PIDConstants.toPID(): PIDController {
    return PIDController(this.p, this.i, this.d)
}

val PIDConstants.PathPlannerPID: com.pathplanner.lib.config.PIDConstants
    get() = com.pathplanner.lib.config.PIDConstants(this.p, this.i, this.d)
