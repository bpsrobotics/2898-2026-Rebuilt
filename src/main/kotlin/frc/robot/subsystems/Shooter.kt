package frc.robot.subsystems

import beaverlib.controls.PIDConstants
import beaverlib.controls.PIDFF
import beaverlib.controls.SimpleMotorFeedForwardConstants
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.asRPM
import beaverlib.utils.Units.Time
import beaverlib.utils.Units.seconds
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.beaverlib.utils.sysID.BeaverSysIDMotor
import frc.robot.beaverlib.utils.sysID.BeaverSysIDRoutine
import frc.robot.subsystems.Shooter.setSpeeds
import kotlin.math.absoluteValue

object Shooter : SubsystemBase() {
    private val topMotor = SparkMax(RobotMap.ShooterTopId, SparkLowLevel.MotorType.kBrushless)
    private val topEncoder = topMotor.encoder
    private val botMotor = SparkMax(RobotMap.ShooterBotId, SparkLowLevel.MotorType.kBrushless)
    private val botEncoder = botMotor.encoder

    private val shooterConfig: SparkMaxConfig = SparkMaxConfig()

    object Constants {
        val botPIDConstants = PIDConstants(0.0015, 0.0, 0.0001)
        val botFFConstants = SimpleMotorFeedForwardConstants(0.20495, 0.0013913, 0.00014398)

        val topPIDConstants = PIDConstants(0.001, 0.0, 0.0001)
        val topFFConstants = SimpleMotorFeedForwardConstants(0.19116, 0.0013576, 0.00011935)
        const val GEAR_RATIO = 3.0 / 2.0
    }

    val topMotorPIDFF: PIDFF = PIDFF(Constants.topPIDConstants, Constants.topFFConstants)
    val botMotorPIDFF: PIDFF = PIDFF(Constants.botPIDConstants, Constants.botFFConstants)

    val topMotorSpeed
        get() = topEncoder.velocity.RPM

    val bottomMotorSpeed
        get() = botEncoder.velocity.RPM

    val speed
        get() = (topMotorSpeed + bottomMotorSpeed) / 2.0

    // val Carrot1 = MechanismLigament2d("Carrot1", 2.5, 0.0, 20.0, Color8Bit(255, 172, 28))

    init {
        // Intake motor initialisation stuff
        shooterConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(20)
            .encoder
            .positionConversionFactor(Constants.GEAR_RATIO)
            .velocityConversionFactor(Constants.GEAR_RATIO)

        topMotor.configure(
            shooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        botMotor.configure(
            shooterConfig.inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        defaultCommand = stopCommand()
    }

    // Creates a SysIdRoutine
    var routine =
        BeaverSysIDRoutine(
            this,
            BeaverSysIDMotor("shooter-bottom", botMotor),
            BeaverSysIDMotor("shooter-top", topMotor),
        )

    /** Runs both shooter motors using openloop at the given [percent] */
    fun runAtPower(percent: Double) {
        topMotor.set(percent)
        botMotor.set(percent)
    }

    /** Stops all motors (top, bottom, and gate) from running */
    fun stop() {
        topMotor.stopMotor()
        botMotor.stopMotor()
    }

    /** Sets the PIDFF setpoint for each motor to [topSpeed] and [bottomSpeed] */
    fun setSpeeds(bottomSpeed: AngularVelocity, topSpeed: AngularVelocity = bottomSpeed) {
        topMotorPIDFF.setpoint = topSpeed.asRPM
        botMotorPIDFF.setpoint = bottomSpeed.asRPM
    }

    /** Runs the shooter using the current setpoint (given by [setSpeeds]) */
    fun runPIDFF() {
        topMotor.setVoltage(topMotorPIDFF.calculate(topEncoder.velocity))
        botMotor.setVoltage(botMotorPIDFF.calculate(botEncoder.velocity))
    }

    /** Returns true if both PIDs [PIDFF.atSetpoint] returns true. */
    fun isAtSpeed(range: AngularVelocity = 50.RPM): Boolean {
        return (botMotor.encoder.velocity - topMotorPIDFF.setpoint).absoluteValue < range.asRPM &&
            (topMotor.encoder.velocity - topMotorPIDFF.setpoint).absoluteValue < range.asRPM
    }

    /** Put the top and bottom motor encoder RPMS to [SmartDashboard] */
    override fun periodic() {
        SmartDashboard.putNumber("Shooter/TopMotorRPM", topEncoder.velocity)
        SmartDashboard.putNumber("Shooter/BottomMotorRPM", botEncoder.velocity)
    }

    /**
     * Runs the Shooter at the specified power for the specified time, intended to get the shooter
     * up to speed before feeding carrots
     */
    fun openloopSpinup(power: Double, time: Time = 0.4.seconds): Command {
        return this.startEnd({ runAtPower(power) }, {}).withTimeout(time.asSeconds)
    }

    /**
     * Runs the Shooter at the specified power for the specified time. If time is null, this will
     * run indefinitely until canceled
     */
    fun openloopShoot(power: Double, time: Time? = null): Command {
        time ?: return this.startEnd({ runAtPower(power) }, { stop() })
        return this.startEnd({ runAtPower(power) }, { stop() }).withTimeout(time.asSeconds)
    }

    /**
     * Attempts to run the Shooter at the specified speed, and will end once that speed is achieved.
     * Intended to get Shooter up to speed before feeding carrots
     */
    fun spinup(
        botSpeed: () -> AngularVelocity,
        topSpeed: () -> AngularVelocity = botSpeed,
        range: AngularVelocity = 50.RPM,
    ): Command {
        return this.run({
                setSpeeds(botSpeed(), topSpeed())
                runPIDFF()
            })
            .until { isAtSpeed(range) }
    }

    /**
     * Attempts to run the Shooter at the specified speed for the specified time. If time is null,
     * this will run indefinitely until canceled
     */
    fun shoot(
        botSpeed: () -> AngularVelocity,
        topSpeed: () -> AngularVelocity = botSpeed,
        time: Time? = null,
    ): Command {
        val command =
            this.run({
                setSpeeds(botSpeed(), topSpeed())
                runPIDFF()
            })
        time ?: return command
        return command.withTimeout(time.asSeconds)
    }

    fun stopCommand(): Command {
        return this.run { stop() }
    }
}
