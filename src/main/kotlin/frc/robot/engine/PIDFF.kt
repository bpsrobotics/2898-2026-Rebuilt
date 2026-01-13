package frc.robot.engine

import beaverlib.controls.PIDConstants
import beaverlib.controls.SimpleMotorFeedForwardConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward

/** A combined Proportional Integral Derivative and Feed Forward controller. */
class PIDFF(PIDConstants: PIDConstants, FFConstants: SimpleMotorFeedForwardConstants) {
    /** The Proportional Integral Derivative controller part of the PIDFF */
    val PID = PIDController(PIDConstants.P, PIDConstants.I, PIDConstants.D)
    /** The FeedForward part of the PIDFF */
    val FeedForward = SimpleMotorFeedforward(FFConstants.kS, FFConstants.kV, FFConstants.kA)

    /** The goal state for the PIDFF */
    var setpoint: Double
        get() = PID.setpoint
        set(value) {
            PID.setpoint = value
        }

    /**
     * Returns the calculated PID value given [measurement], plus the calculated FeedForwardValue
     * given the [setpoint]
     *
     * @param measurement The measured value of what the PIDFF controls
     */
    fun calculate(measurement: Double): Double {
        return PID.calculate(measurement) + FeedForward.calculate(setpoint)
    }

    /** Returns this PID [PIDController.atSetpoint] */
    fun atSetpoint(): Boolean {
        return PID.atSetpoint()
    }
}
