package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Time
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkAbsoluteEncoder
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object Intake : SubsystemBase() {

    object Pivot : SubsystemBase() {

        // Initializing brushless motor with SparkMAX motor controller
        private val motor = SparkMax(RobotMap.PivotID, SparkLowLevel.MotorType.kBrushless)
        private val motorConfig: SparkMaxConfig = SparkMaxConfig()

        // Values for PID tuning
        private const val kP: Double = 0.0
        private const val kI: Double = 0.0
        private const val kD: Double = 0.0
        val absEncoder: SparkAbsoluteEncoder = motor.getAbsoluteEncoder()

        init {
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            defaultCommand = stopCommand()
        }

        private val PIDController = PIDController(kP, kI, kD)

        override fun periodic() {
            val PIDOutput: Double = PIDController.calculate(absEncoder.position)
            motor.set(PIDOutput)
        }

        fun calculatePID(): Double {
            return (PIDController.calculate(absEncoder.position))
        }

        fun runMotor(speed: Double = 0.0, isPID: Boolean) {
            motor.set(if (isPID) PIDController.calculate(absEncoder.position) else speed)
        }
    }

    private val motor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

    object Hopper : SubsystemBase() {
        private val motor = SparkMax(RobotMap.HopperID, SparkLowLevel.MotorType.kBrushless)
        private val motorConfig: SparkMaxConfig = SparkMaxConfig()

        init {
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            defaultCommand = stopCommand()
        }

        // Periodic hopper diagnostic outputs
        override fun periodic() {
            SmartDashboard.putNumber("Hopper motor voltage:", motor.busVoltage)
            SmartDashboard.putNumber("Hopper motor current usage:", motor.outputCurrent)
            SmartDashboard.putNumber("Hopper motor temperature:", motor.motorTemperature)
        }

        fun runAtPower(power: Double) {
            motor.set(power)
        }

        fun stop() {
            motor.stopMotor()
        }
    }

    init {
        // Intake motor initialisation stuff
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        defaultCommand = stopCommand()
    }

    val speed: AngularVelocity
        get() = motor.encoder.velocity.RPM

    override fun periodic() {
        SmartDashboard.putNumber("Intake/motorVoltage", motor.busVoltage)
        SmartDashboard.putNumber("Intake/motorCurrent", motor.outputCurrent)
        SmartDashboard.putNumber("Intake motor temperature:", motor.motorTemperature)
    }

    /**
     * Run the Intake at the given speed
     *
     * @param percent (-1, 1) the percent speed to run the motor at
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
    fun runAtPowerCommand(power: Double, time: Time? = null): Command {
        time ?: return this.runEnd({ runAtPower(power) }, { stop() })
        return this.runEnd({ runAtPower(power) }, { stop() }).withTimeout(time.asSeconds)
    }

    /** Command that stops the Intake motor */
    fun stopCommand() = this.run { stop() }
}
