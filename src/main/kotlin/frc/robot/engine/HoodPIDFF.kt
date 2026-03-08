package frc.robot.engine

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.PIDConstants
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.sign

/**
 * A combined Proportional Integral Derivative and an Arm Feed Forward controller.
 *
 * @param pidConstants The P I and D values to use for this controller
 * @param ffConstants The arm feedforward constants to use for this controller
 * @param zeroPosition the angle at which the endEffector is level with the ground (Where gravity
 *   will affect it the most)
 *     @param kC constant voltage to be applied to the motor, regardless of any other factor
 */
open class HoodPIDFF(
    pidConstants: PIDConstants,
    ffConstants: ArmFeedForwardConstants,
    var zeroPosition: AngleUnit = 0.0.radians,
    var kC: Double = 0.0,
) : Sendable {
    /** The Proportional Integral Derivative controller part of the PidFF */
    val pid = PIDController(pidConstants.p, pidConstants.i, pidConstants.d)
    var kS = ffConstants.kS
    var kG = ffConstants.kG
    var kV = ffConstants.kV
    var kA = ffConstants.kA

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
        desiredVelocity: AngularVelocity = 0.radiansPerSecond,
    ): Double {
        var voltage = pid.calculate(measurement.asRadians)
        if (desiredVelocity != 0.radiansPerSecond) {
            voltage += sign(desiredVelocity.asRadiansPerSecond) * kS
        } else if (!pid.atSetpoint()) voltage += sign(voltage) * kS
        voltage += (setpoint - zeroPosition).cos() * kG
        return voltage +
            (kV * desiredVelocity.asRadiansPerSecond) +
            (kA * desiredVelocity.asRadiansPerSecond)
    }

    /**
     * Returns the calculated PID value given [measurement], plus the calculated FeedForwardValue
     * given the [setpoint]
     *
     * @param measurement The measured value of what the PidFF controls
     */
    open fun test(measurement: AngleUnit): Double {
        val voltage =
            pid.calculate(measurement.asRadians) + kS + (setpoint - zeroPosition).cos() * kG
        return voltage
    }

    /** Returns this PID [PIDController.atSetpoint] */
    fun atSetpoint(): Boolean {
        return pid.atSetpoint()
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("kS", { kS }, { kS = it })
        builder.addDoubleProperty("kG", { kG }, { kG = it })
        builder.addDoubleProperty("kV", { kV }, { kV = it })
        builder.addDoubleProperty("kA", { kA }, { kA = it })
        builder.addDoubleProperty("kC", { kC }, { kC = it })
        builder.addDoubleProperty("kP", pid::getP, pid::setP)
        builder.addDoubleProperty("kI", pid::getI, pid::setI)
        builder.addDoubleProperty("kD", pid::getD, pid::setD)
        builder.addDoubleProperty("setpoint", { setpoint.asRadians }, { setpoint = it.radians })
    }
}
