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

    private val motor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

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

    object Pivot : SubsystemBase() {

        private const val kP: Double = 0.0  // Proportional
        private const val kI: Double = 0.0  // Integral
        private const val kD: Double = 0.0  // Derivative

        // Initializing brushless motor with SparkMAX motor controller
        private val motor = SparkMax(RobotMap.PivotID, SparkLowLevel.MotorType.kBrushless)
        private val motorConfig: SparkMaxConfig = SparkMaxConfig()

        // Use encoder values for PID tuning
        val absEncoder: SparkAbsoluteEncoder = motor.getAbsoluteEncoder()

        init {
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
            motor.configure(
                motorConfig,
                // The reset mote and persist mode have to do with maintaining
                // settings after a power cycle.
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            // Stop if nothing else is going on
            defaultCommand = stopCommand()
        }

        // PID controller class for pivot subsystem
        private val PIDController = PIDController(kP, kI, kD)

        // Periodically use the PID controller output as value to pass to the motor.
        override fun periodic() {
            val PIDOutput: Double = calculatePID()
            motor.set(PIDOutput)
        }


        fun calculatePID(): Double {
            // Use absolute encoder as the feedback sensor for the PID controller
            return (PIDController.calculate(absEncoder.position))
        }

        // If isPID is false, then the raw speed will be used rather than
        // a PID adjusted one. Not recommended for this case.
        fun runMotor(speed: Double = 0.0, isPID: Boolean = true) {
            motor.set(if (isPID) calculatePID() else speed)
        }
    }
}
