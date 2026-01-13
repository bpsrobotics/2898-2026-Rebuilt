package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Time
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object Intake : SubsystemBase() {
    private val motor = SparkMax(RobotMap.IntakeId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

    init {
        // Intake motor initialisation stuff
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40).inverted(true)

        motor.configure(
            motorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
        defaultCommand = stopCommand()
    }

    val speed: AngularVelocity
        get() = motor.encoder.velocity.RPM

    override fun periodic() {
        SmartDashboard.putNumber("Intake/motorVoltage", motor.busVoltage)
        SmartDashboard.putNumber("Intake/motorCurrent", motor.outputCurrent)
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
