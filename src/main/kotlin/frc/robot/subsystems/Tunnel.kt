package frc.robot.subsystems

import beaverlib.utils.Units.Angular.rotations
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Time
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

class Carrot(var offset: DistanceUnit) {
    companion object {
        val averageLength = 7.25.inches
        val tolerance = 0.45.inches
    }

    val position
        get() = Tunnel.beltPosition - offset

    val inSensorRange: Boolean
        get() = (position < tolerance) && position > averageLength + tolerance
}

object Tunnel : SubsystemBase() {
    private val motor = SparkMax(RobotMap.TunnelId, SparkLowLevel.MotorType.kBrushless)
    private val motorConfig: SparkMaxConfig = SparkMaxConfig()

    /*private val intakeDetector: ObjectSensor = //LaserSharkObjectSensor(IntakeSharkId, 2.inches)
    private val feederDetector: ObjectSensor = //LaserSharkObjectSensor(FeederSharkId, 2.inches)

    object CarrotCounter {
        val carrots: MutableList<Carrot> = mutableListOf()

        fun carrotInSensorRange(): Boolean {
            carrots.forEach { carrot -> if (carrot.inSensorRange) return true }
            return false
        }

        fun update() {
            var blockage = Constants.beltLength
            carrots.forEach { carrot ->
                if (carrot.position > blockage) carrot.offset = beltPosition - blockage
                blockage -= Carrot.averageLength
            }
        }

        fun shootCarrot() {
            carrots.removeAt(0)
            carrots.forEach { carrot -> carrot.offset += Carrot.averageLength }
        }
    }*/

    object Constants {
        val Diameter = 6.inches // todo
        val beltLength = 20.inches
    }

    val beltPosition: DistanceUnit = motor.encoder.position.rotations * Constants.Diameter

    init {
        // Intake motor initialisation stuff
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        defaultCommand = stopCommand()
    }

    fun runAtPower(power: Double) {
        motor.set(power)
    }

    fun stop() {
        motor.stopMotor()
    }

    override fun periodic() {
        // CarrotCounter.update()
    }

    /**
     * Runs the Tunnel at the specified power for the specified time. If time is null, this will run
     * indefinitely until canceled
     */
    fun runAtPowerCommand(power: Double, time: Time? = null): Command {
        time ?: return this.runEnd({ runAtPower(power) }, { stop() })
        return this.runEnd({ runAtPower(power) }, { stop() }).withTimeout(time.asSeconds)
    }

    /** Command that stops the Tunnel motor */
    fun stopCommand(): Command = this.run { stop() }
}
