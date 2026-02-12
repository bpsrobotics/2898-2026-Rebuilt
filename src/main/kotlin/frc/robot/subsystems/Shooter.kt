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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {
    private object Constants {
        // TODO(ant): Get CAN IDs from Electrical
        const val MOTOR_1_ID = 70
        const val MOTOR_2_ID = 71

        val motor1PIDConstants = PIDConstants(0.0, 0.0, 0.0)
        val motor2PIDConstants = PIDConstants(0.0, 0.0, 0.0)

        val motor1FFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)
        val motor2FFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)
    }

    private val motor1 = SparkMax(Constants.MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless)
    private val motor2 = SparkMax(Constants.MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless)

    private val motor1PIDFF = PIDFF(Constants.motor1PIDConstants, Constants.motor1FFConstants)
    private val motor2PIDFF = PIDFF(Constants.motor2PIDConstants, Constants.motor2FFConstants)

    init {
        val shooterConfig = SparkMaxConfig()
        shooterConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(20)
            .encoder
            .velocityConversionFactor(2.0)
        motor1.configure(
            shooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        motor2.configure(
            shooterConfig.inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        defaultCommand = stop()
    }

    fun stop(): Command {
        return runOnce {
            motor1.stopMotor()
            motor2.stopMotor()
        }
    }

    fun runSpeed(setpointAngVelocity: Double): Command {
        return run {
            motor1PIDFF.setpoint = setpointAngVelocity
            motor1.set(motor1PIDFF.calculate(motor1.encoder.velocity))

            motor2PIDFF.setpoint = setpointAngVelocity
            motor2.set(motor2PIDFF.calculate(motor2.encoder.velocity))
        }
    }

    fun waitSpeed(setpointAngVelocity: Double): Command {
        return ParallelRaceGroup(
            runSpeed(setpointAngVelocity),
            waitUntil({ motor1PIDFF.atSetpoint() && motor2PIDFF.atSetpoint() }),
        )
    }
}
