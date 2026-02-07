package frc.robot.subsystems

import beaverlib.utils.Units.Angular.radiansPerSecond
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Shooter.Constants.botkD
import frc.robot.subsystems.Shooter.Constants.botkI
import frc.robot.subsystems.Shooter.Constants.botkP
import frc.robot.subsystems.Shooter.Constants.bottomencoder
import frc.robot.subsystems.Shooter.Constants.maxAngularAcceleration
import frc.robot.subsystems.Shooter.Constants.maxAngularvelocity
import frc.robot.subsystems.Shooter.Constants.tempIDbottom
import frc.robot.subsystems.Shooter.Constants.topencoder
import frc.robot.subsystems.Shooter.Constants.topkD
import frc.robot.subsystems.Shooter.Constants.topkI
import frc.robot.subsystems.Shooter.Constants.topkP
import frc.robot.subsystems.Shooter.Constants.toptempID

object Shooter : SubsystemBase() {
    object Constants {
        val maxAngularvelocity = 60.0 // radians/second
        val maxAngularAcceleration = 10.0 // radians/second^2
        val toptempID = 0
        val tempIDbottom = 1
        val topencoder = 99
        val bottomencoder = 98
        val topkP = 0.0
        val topkI = 0.0
        val topkD = 0.0
        val botkP = 0.0
        val botkI = 0.0
        val botkD = 0.0
        // todo ensure all values for velocity are in rads/sec
    }

    val topMotor = SparkMax(toptempID, SparkLowLevel.MotorType.kBrushless)
    val bottomMotor = SparkMax(tempIDbottom, SparkLowLevel.MotorType.kBrushless)

    private var ShooterConfig: SparkMaxConfig = SparkMaxConfig()
    private val Encoder = Encoder(topencoder, bottomencoder)
    private val constraints =
        TrapezoidProfile.Constraints(maxAngularvelocity, maxAngularAcceleration)
    private var profile = TrapezoidProfile(constraints)
    var currentState =
        TrapezoidProfile.State(
            Encoder.distance,
            0.0,
        ) // todo configure this to take actual motor speeds
    var goalState = TrapezoidProfile.State(Encoder.distance, 0.0)
    val TopPIDController = PIDController(topkP, topkI, topkD)
    val TopFF: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)
    val BotPIDController = PIDController(botkP, botkI, botkD)
    val BotFF: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    // Shooter speed:
    val topMotorSpeed
        // you can also define motor speed using angular setpoints
        get() = topencoder.radiansPerSecond

    val bottomMotorSpeed
        // you can also define motor speed using angular setpoints
        get() = bottomencoder.radiansPerSecond

    init {
        ShooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(40)
            .encoder
            .velocityConversionFactor(1.0)
        topMotor.configure(
            ShooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        bottomMotor.configure(
            ShooterConfig.inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
    }

    fun calculateVoltage() {
        topMotor.set(
            TopPIDController.calculate(currentState.velocity, goalState.velocity) +
                TopFF.calculate(goalState.velocity)
        )
        bottomMotor.set(
            BotPIDController.calculate(currentState.velocity, goalState.velocity) +
                BotFF.calculate(goalState.velocity)
        )
    }
}
