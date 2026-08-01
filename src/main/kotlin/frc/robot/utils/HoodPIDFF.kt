package frc.robot.utils

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.utils.controls.ArmFeedForwardConstants
import frc.robot.utils.controls.PIDConstants
import kotlin.math.cos
import kotlin.math.sign

/**
 * A combined Proportional Integral Derivative and an Arm Feed Forward controller.
 *
 * @param pidConstants The P I and D values to use for this controller
 * @param ffConstants The arm feedforward constants to use for this controller
 * @param zeroPosition the angle at which the endEffector is level with the ground (Where gravity
 *   will affect it the most)
 * @param kC constant voltage to be applied to the motor, regardless of any other factor
 */
open class HoodPIDFF(
    pidConstants: PIDConstants,
    ffConstants: ArmFeedForwardConstants,
    var zeroPosition: Angle = 0.0.radians,
    var kC: Double = 0.0,
) : Sendable {
    /** The Proportional Integral Derivative controller part of the PidFF */
    val pid = PIDController(pidConstants.p, pidConstants.i, pidConstants.d)
    var kS = ffConstants.kS
    var kG = ffConstants.kG
    var kV = ffConstants.kV
    var kA = ffConstants.kA
    var kGVoltage = 0.0
    var kSVoltage = 0.0

    /** The goal state for the PidFF */
    var setpoint: Angle
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
        measurement: Angle,
        desiredVelocity: AngularVelocity = 0.radiansPerSecond,
    ): Double {
        var voltage = pid.calculate(measurement.asRadians)
        if (!pid.atSetpoint()) {
            kSVoltage = sign(voltage) * kS
            voltage += sign(voltage) * kS
        }
        voltage += cos((setpoint - zeroPosition).convert(Units.Radians)) * kG
        kGVoltage = cos((setpoint - zeroPosition).convert(Units.Radians)) * kG
        return voltage +
            (kV * desiredVelocity.convert(Units.RadiansPerSecond)) +
            (kA * desiredVelocity.convert(Units.RadiansPerSecond))
    }

    /**
     * Returns the calculated PID value given [measurement], plus the calculated FeedForwardValue
     * given the [setpoint]
     *
     * @param measurement The measured value of what the PidFF controls
     */
    open fun test(measurement: Angle): Double {
        val voltage =
            pid.calculate(measurement.asRadians) +
                kS +
                cos((setpoint - zeroPosition).convert(Units.Radians)) * kG
        return voltage
    }

    /** Returns this PID [PIDController.atSetpoint] */
    fun atSetpoint(): Boolean {
        return pid.atSetpoint()
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("kS", { kS }, { kS = it })
        builder.addDoubleProperty("kGVoltage", { kGVoltage }, { kS = it })
        builder.addDoubleProperty("kSVoltage", { kSVoltage }, { kS = it })

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
