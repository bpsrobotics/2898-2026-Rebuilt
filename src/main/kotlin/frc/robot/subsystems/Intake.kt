package frc.robot.subsystems

import beaverlib.controls.PIDConstants
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Time
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
import frc.robot.RobotMap
import frc.robot.engine.toPIDSin
import kotlin.math.PI

object Intake : SubsystemBase() {

    object Constants {
        val pivotPID: PIDConstants = PIDConstants(0.0, 0.0, 0.0)
        val pivotkSin: Double = 0.0
        val pivotStowedPosition = 0.degrees
        val pivotExtendedPosition = PI.radians
    }

    private val motor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

    init {
        // Intake motor initialisation stuff
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        defaultCommand = doStop()
    }

    val speed: AngularVelocity
        get() = motor.encoder.velocity.RPM

    override fun periodic() {
        SmartDashboard.putNumber("Intake/motorCurrent", motor.outputCurrent)
        SmartDashboard.putNumber("Intake motor temperature:", motor.motorTemperature)
    }

    /**
     * Run the Intake at the given speed
     *
     * @param power (-1, 1) the percent speed to run the motor at
     */
    fun runAtPower(power: Double) {
        motor.set(power)
    }

    /** Stops the Intake motor */
    fun stop() {
        motor.stopMotor()
    }

    /**
     * Runs the Intake at the specified power for the specified time. If time is null, this will run
     * indefinitely until canceled
     */
    fun doRunAtPower(power: Double, time: Time? = null): Command {
        time ?: return this.runEnd({ runAtPower(power) }, { stop() })
        return this.runEnd({ runAtPower(power) }, { stop() }).withTimeout(time.asSeconds)
    }

    /** Command that stops the Intake motor */
    fun doStop(): Command = this.run { stop() }

    object Pivot : SubsystemBase() {
        // PID controller class for pivot subsystem
        private val PIDSinController = Constants.pivotPID.toPIDSin(Constants.pivotkSin)

        // Initializing brushless motor with SparkMAX motor controller
        private val motor = SparkMax(RobotMap.PivotID, SparkLowLevel.MotorType.kBrushless)
        private val motorConfig: SparkMaxConfig = SparkMaxConfig()

        // Use encoder values for PID tuning
        val absEncoder: DutyCycleEncoder = DutyCycleEncoder(0)

        init {
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                // The reset mote and persist mode have to do with maintaining
                // settings after a power cycle.
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            SmartDashboard.putData("Intake/PivotPID", PIDSinController.PID)

            // Stabilize the wrist if nothing else is happening
            defaultCommand = doStabilize()
        }

        /** Stops the wrist */
        fun doStop(): Command = this.run { motor.stopMotor() }

        /** Holds the wrist at the last set position */
        fun doStabilize(): Command =
            this.run {
                motor.setVoltage(
                    PIDSinController.calculate(absEncoder.get().radians).clamp(-1.0, 1.0)
                )
            }

        /** Sets the wrist to target position, and ends once the PID is at the setpoint */
        fun doSetPosition(targetPosition: AngleUnit): Command =
            doStabilize()
                .beforeStarting(
                    InstantCommand({ PIDSinController.setpoint = targetPosition.asRadians })
                )
                .until { PIDSinController.PID.atSetpoint() }
    }
}
