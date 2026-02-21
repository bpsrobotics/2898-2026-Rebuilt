package frc.robot.subsystems

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.ArmPIDFF
import beaverlib.controls.PIDConstants
import beaverlib.controls.PIDFF
import beaverlib.controls.SimpleMotorFeedForwardConstants
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.rotations
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

    fun runPIDFF(): Command = run {
        motor1.set(motor1PIDFF.calculate(motor1.encoder.velocity))
        motor2.set(motor2PIDFF.calculate(motor2.encoder.velocity))
    }

    fun waitSpeed(): Command = waitUntil { motor1PIDFF.atSetpoint() && motor2PIDFF.atSetpoint() }

    fun doRunAtSpeed(): Command =
        startRun({
            motor1PIDFF.setpoint = Constants.runningSpeed
            motor2PIDFF.setpoint = Constants.runningSpeed
        }) {
            runPIDFF()
        }

    fun doRunAtPower(power: Double): Command = run {
        motor1.set(power)
        motor2.set(power)
    }

    object Hood : SubsystemBase() {
        private object Constants {
            const val MOTOR_ID = 18
            const val ENCODER_ID = 1

            val pidConstants = PIDConstants(0.0, 0.0, 0.0)
            val ffConstants = ArmFeedForwardConstants(0.0, 0.0, 0.0)

            val DOWN_POSITION = 0.0.radians
        }

        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        private val controller = ArmPIDFF(Constants.pidConstants, Constants.ffConstants)

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            defaultCommand = doStop() // todo doMoveDown()

            SmartDashboard.putData("Shooter/Hood/PID", controller.PID)
            // SmartDashboard.putData("Shooter/Hood/FF", FFSendable(controller.FeedForward))
        }

        fun doStop(): Command = run {
            motor1.stopMotor()
            motor2.stopMotor()
        }

        fun doHoldPosition(position: AngleUnit): Command =
            startRun({ controller.setpoint = position }) {
                motor.setVoltage(controller.calculate(absEncoder.get().rotations))
            }

        fun doMoveToPosition(position: AngleUnit): Command =
            doHoldPosition(position).withDeadline(waitUntil { controller.atSetpoint() })

        fun doMoveDown() = doHoldPosition(Constants.DOWN_POSITION)
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

            defaultCommand = doStop()

            SmartDashboard.putData("Shooter/Feeder/PID", controller.PID)
            SmartDashboard.putData("Shooter/Feeder/FF", FFSendable(controller.FeedForward))
        }

        fun doStop(): Command = runOnce { motor.stopMotor() }

        fun doRunAtPower(power: Double): Command = run { motor.set(power) }

        fun doRunAtSpeed(): Command =
            startRun({ controller.setpoint = Constants.runningSpeed }) {
                motor.set(controller.calculate(motor.encoder.velocity))
            }
    }
}
