package frc.robot.subsystems

import beaverlib.controls.PIDConstants
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.rotations
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.engine.toPIDSin
import kotlin.math.PI

object Intake : SubsystemBase() {
    private object Constants {
        // TODO: Get CAN IDs from Electrical
        const val MOTOR_ID = 13
    }

    private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    init {
        // Intake motor initialization stuff
        val motorConfig = SparkMaxConfig()
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        defaultCommand = stop()
    }

    override fun periodic() {
        SmartDashboard.putNumber("Intake/motorCurrent", motor.outputCurrent)
        SmartDashboard.putNumber("Intake motor temperature:", motor.motorTemperature)
    }

    /**
     * Run the Intake at the given speed
     *
     * @param power (-1, 1) the portion of max speed to run the motor at
     */
    fun runAtPower(power: Double): Command = runEnd({ motor.set(power) }, { motor.stopMotor() })

    /** Stops the Intake motor */
    fun stop(): Command = runOnce { motor.stopMotor() }

    object Pivot : SubsystemBase() {
        private object Constants {
            // TODO(ant): Get CAN IDs from Electrical
            const val PIVOT_ID = 15
            const val ENCODER_ID = 0

            val pidConstants: PIDConstants = PIDConstants(0.67, 0.0, 0.0)
            const val K_SIN: Double = 0.98
            val PIVOT_STOWED_POSITION = 0.degrees
            val PIVOT_EXTENDED_POSITION = PI.radians
        }

        // Initializing brushless motor with SparkMAX motor controller
        private val motor = SparkMax(Constants.PIVOT_ID, SparkLowLevel.MotorType.kBrushless)

        // Use encoder values for PID tuning
        private val absEncoder: DutyCycleEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        // PID controller class for pivot subsystem
        private val pidSin = Constants.pidConstants.toPIDSin(Constants.K_SIN)

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                // TODO: Tune current limit but Recalc says we need at least 30A
                .smartCurrentLimit(15)
            motor.configure(
                motorConfig,
                // The reset mote and persist mode have to do with maintaining
                // settings after a power cycle.
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            SmartDashboard.putData("Intake/Pivot/PIDSin", pidSin)

            // Stabilize the wrist if nothing else is happening
            defaultCommand = stabilize()
        }

        /** Stops the wrist */
        fun stop(): Command = runOnce { motor.stopMotor() }

        /** Holds the wrist at the last set position */
        fun stabilize(): Command = run {
            motor.setVoltage(pidSin.calculate(absEncoder.get().rotations).clamp(-1.0, 1.0))
        }

        /** Sets the wrist to target position, and ends once the PID is at the setpoint */
        fun runToPosition(targetPosition: AngleUnit): Command =
            stabilize().beforeStarting(InstantCommand({ pidSin.setpoint = targetPosition })).until {
                pidSin.pid.atSetpoint()
            }

        fun extend() = runToPosition(Constants.PIVOT_EXTENDED_POSITION)

        fun stow() = runToPosition(Constants.PIVOT_STOWED_POSITION)
    }
}
