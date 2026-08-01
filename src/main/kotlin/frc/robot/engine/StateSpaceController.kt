package frc.robot.engine

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.Num
import edu.wpi.first.math.Vector
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.TimedRobot

class StateSpaceController<States : Num, Inputs : Num, Outputs : Num>(
    states: Nat<States>,
    outputs: Nat<Outputs>,
    val plant: LinearSystem<States, Inputs, Outputs>,
    stateStdDevs: Matrix<States, N1>,
    measurementStdDevs: Matrix<Outputs, N1>,
    stateTolerance: Vector<States>,
    controlTolerance: Vector<Inputs>,
    dtSeconds: Double = TimedRobot.kDefaultPeriod,
    maxVoltage: Voltage = Volts.of(12.0),
) {
    val observer =
        KalmanFilter<States, Inputs, Outputs>(
            states,
            outputs,
            plant,
            stateStdDevs,
            measurementStdDevs,
            dtSeconds,
        )
    val lqr =
        LinearQuadraticRegulator<States, Inputs, Outputs>(
            plant,
            stateTolerance,
            controlTolerance,
            dtSeconds,
        )
    val loop = LinearSystemLoop(plant, lqr, observer, maxVoltage.`in`(Volts), dtSeconds)
}
