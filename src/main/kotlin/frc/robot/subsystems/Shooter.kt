package frc.robot.subsystems

import beaverlib.controls.PIDConstants
import beaverlib.controls.PIDFF
import beaverlib.controls.SimpleMotorFeedForwardConstants
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Shooter.Constants.tempIDbottom
import frc.robot.subsystems.Shooter.Constants.toptempID

object Shooter : SubsystemBase() {
    object Constants {
        val maxAngularvelocity = 60.0 // radians/second
        val maxAngularAcceleration = 10.0 // radians/second^2
        val toptempID = 0
        val tempIDbottom = 1
        val topencoder = 99
        val bottomencoder = 98

        val topPIDConstants = PIDConstants(0.0, 0.0, 0.0)
        val bottomPIDConstants = PIDConstants(0.0, 0.0, 0.0)

        val topFFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)
        val bottomFFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)

        // todo ensure all values for velocity are in rads/sec
    }

    val topMotor = SparkMax(toptempID, SparkLowLevel.MotorType.kBrushless)
    val bottomMotor = SparkMax(tempIDbottom, SparkLowLevel.MotorType.kBrushless)

    private var ShooterConfig: SparkMaxConfig = SparkMaxConfig()

    val TopPIDFF = PIDFF(Constants.topPIDConstants, Constants.topFFConstants)
    val BottomPIDFF = PIDFF(Constants.bottomPIDConstants, Constants.bottomFFConstants)

    init {
        ShooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(20)
            .encoder
            .velocityConversionFactor(2.0)
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

    fun doMaintainSpeed() = this.run {}
}
