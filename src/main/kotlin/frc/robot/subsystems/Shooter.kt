package frc.robot.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Shooter.Constants.bottomencoder
import frc.robot.subsystems.Shooter.Constants.maxAngularAcceleration
import frc.robot.subsystems.Shooter.Constants.maxAngularvelocity
import frc.robot.subsystems.Shooter.Constants.tempIDbottom
import frc.robot.subsystems.Shooter.Constants.topencoder
import frc.robot.subsystems.Shooter.Constants.toptempID

object Shooter : SubsystemBase() {
    object Constants {
        val maxAngularvelocity = 60.0 // radians/second
        val maxAngularAcceleration = 10.0 // radians/second^2
        val toptempID = 0
        val tempIDbottom = 1
        val topencoder = 99
        val bottomencoder = 98
        private val kP = 0.0
        private val kI = 0.0
        private val kD = 0.0
    }

    val topMotor = SparkMax(toptempID, SparkLowLevel.MotorType.kBrushless)
    val bottomMotor = SparkMax(tempIDbottom, SparkLowLevel.MotorType.kBrushless)

    private var ShooterConfig: SparkMaxConfig = SparkMaxConfig()
    private val Encoder = Encoder(topencoder, bottomencoder)
    private val constraints =
        TrapezoidProfile.Constraints(maxAngularvelocity, maxAngularAcceleration)
    private var profile = TrapezoidProfile(constraints)
    var currentState = TrapezoidProfile.State(Encoder.distance, 0.0)
    var goalState = TrapezoidProfile.State(Encoder.distance, 0.0)

    // Shooter speed:
    val topMotorSpeed
        // you can also define motor speed using angular setpoints
        get() = topencoder.velocity.RPM
    val bottomMotorSpeed
        // you can also define motor speed using angular setpoints
        get() = bottomencoder.velocity.RPM
}
