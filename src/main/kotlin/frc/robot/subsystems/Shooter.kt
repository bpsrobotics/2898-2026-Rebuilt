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
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.engine.DashboardNumber
import frc.robot.engine.FFSendable

object Shooter : SubsystemBase() {
    private object Constants {
        const val MOTOR_1_ID = 16
        const val MOTOR_2_ID = 17

        val motor1PIDConstants = PIDConstants(0.47, 0.0, 0.0)
        val motor2PIDConstants = PIDConstants(0.47, 0.0, 0.0)

        val motor1FFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)
        val motor2FFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)

        val runningSpeed by DashboardNumber(0.0, "Shooter/Constants")
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

        SmartDashboard.putData("Shooter/motor1/PID", motor1PIDFF.PID)
        SmartDashboard.putData("Shooter/motor1/FF", FFSendable(motor1PIDFF.FeedForward))
        SmartDashboard.putData("Shooter/motor2/PID", motor2PIDFF.PID)
        SmartDashboard.putData("Shooter/motor2/FF", FFSendable(motor2PIDFF.FeedForward))
    }

    fun stop(): Command = runOnce {
        motor1.stopMotor()
        motor2.stopMotor()
    }

    fun runSpeed(): Command = run {
        motor1PIDFF.setpoint = Constants.runningSpeed
        motor1.set(motor1PIDFF.calculate(motor1.encoder.velocity))

        motor2PIDFF.setpoint = Constants.runningSpeed
        motor2.set(motor2PIDFF.calculate(motor2.encoder.velocity))
    }

    fun waitSpeed(): Command = waitUntil { motor1PIDFF.atSetpoint() && motor2PIDFF.atSetpoint() }

    object Hood : SubsystemBase() {
        private object Constants {
            const val MOTOR_ID = 18
            const val ENCODER_ID = 1

            val pidConstants = PIDConstants(0.0, 0.0, 0.0)
            val ffConstants = SimpleMotorFeedForwardConstants(0.0, 0.0, 0.0)

            const val DOWN_POSITION = 0.0
        }

        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        private val controller = PIDFF(Constants.pidConstants, Constants.ffConstants)

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            defaultCommand = down()

            SmartDashboard.putData("Shooter/Hood/PID", controller.PID)
            SmartDashboard.putData("Shooter/Hood/FF", FFSendable(controller.FeedForward))
        }

        fun holdPosition(position: Double): Command = run {
            controller.setpoint = position
            motor.set(controller.calculate(absEncoder.get()))
        }

        fun toPosition(position: Double): Command =
            holdPosition(position).withDeadline(waitUntil { controller.atSetpoint() })

        fun down() = toPosition(Constants.DOWN_POSITION)
    }

    object Feeder : SubsystemBase() {
        private object Constants {
            const val MOTOR_ID = 15

            val pidConstants = PIDConstants(0.0, 0.0, 0.0)
            val ffConstants = SimpleMotorFeedForwardConstants(0.0, 0.0, 0.0)

            val runningSpeed by DashboardNumber(100.0, "Shooter/Feeder/Constants")
        }

        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val controller = PIDFF(Constants.pidConstants, Constants.ffConstants)

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            defaultCommand = stop()

            SmartDashboard.putData("Shooter/Feeder/PID", controller.PID)
            SmartDashboard.putData("Shooter/Feeder/FF", FFSendable(controller.FeedForward))
        }

        fun stop(): Command = runOnce { motor.stopMotor() }

        fun runSpeed(): Command = run {
            controller.setpoint = Constants.runningSpeed
            motor.set(controller.calculate(motor.encoder.velocity))
        }
    }
}
