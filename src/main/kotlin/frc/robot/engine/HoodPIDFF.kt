package frc.robot.engine

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.PIDConstants
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.absoluteValue

/** A combined Proportional Integral Derivative and an Arm Feed Forward controller. */
open class HoodPIDFF(pidConstants: PIDConstants, ffConstants: ArmFeedForwardConstants) : Sendable {
    /** The Proportional Integral Derivative controller part of the PidFF */
    val pid = PIDController(pidConstants.p, pidConstants.i, pidConstants.d)
    /** The FeedForward part of the PidFF */
    val feedforward = ArmFeedforward(ffConstants.kS, 0.0, ffConstants.kV, ffConstants.kA)
    var kG = ffConstants.kG

    /** The goal state for the PidFF */
    var setpoint: AngleUnit
        get() = pid.setpoint.radians
        set(value) {
            pid.setpoint = value.asRadians
        }

    /**
     * Returns the calculated PID value given [measurement], plus the calculated FeedForwardValue
     * given the [setpoint]
     *
     * @param measurement The measured value of what the PidFF controls
     */
    open fun calculate(
        measurement: AngleUnit,
        velocity: AngularVelocity = 0.radiansPerSecond,
    ): Double {
        return pid.calculate(measurement.asRadians) +
            feedforward.calculate(measurement.asRadians, velocity.asRadiansPerSecond) +
            (kG * (setpoint - 22.degrees).sin().absoluteValue)
    }

    /** Returns this PID [PIDController.atSetpoint] */
    fun atSetpoint(): Boolean {
        return pid.atSetpoint()
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("kS", feedforward::getKs, feedforward::setKs)
        builder.addDoubleProperty("kG", { kG }, { v: Double -> kG = v })
        builder.addDoubleProperty("kV", feedforward::getKv, feedforward::setKv)
        builder.addDoubleProperty("kA", feedforward::getKa, feedforward::setKa)
        builder.addDoubleProperty("kP", pid::getP, pid::setP)
        builder.addDoubleProperty("kI", pid::getI, pid::setI)
        builder.addDoubleProperty("kD", pid::getD, pid::setD)
        builder.addDoubleProperty("setpoint", { setpoint.asRadians }, { setpoint = it.radians })
    }
}
