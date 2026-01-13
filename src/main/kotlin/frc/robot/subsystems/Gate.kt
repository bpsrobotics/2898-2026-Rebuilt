package frc.robot.subsystems

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Time
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.RobotMap

object Gate : Subsystem {
    private val motor = SparkMax(RobotMap.FeederId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

    object Constants {
        const val GEAR_RATIO = 3.0 / 2.0
    }

    init {
        // Intake motor initialisation stuff
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20).inverted(true)
        motor.configure(
            motorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        defaultCommand = stopCommand()
    }

    val gateSpeed
        get() = motor.encoder.velocity.RPM

    /** Runs the gate motor using openloop at the given [power] */
    fun runAtPower(power: Double) {
        motor.set(power)
    }

    /** Sets the gate motor to stop running */
    fun stop() {
        motor.stopMotor()
    }

    /**
     * Runs the gate at the specified power for the specified time. If time is null, this will run
     * indefinitely until canceled
     */
    fun runAtPowerCommand(power: Double, time: Time? = null): Command {
        time ?: return this.startEnd({ runAtPower(power) }, { stop() })
        return this.startEnd({ runAtPower(power) }, { stop() }).withTimeout(time.asSeconds)
    }

    fun stopCommand() = this.run { stop() }
}
