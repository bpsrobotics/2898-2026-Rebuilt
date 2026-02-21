package frc.robot.subsystems

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.ArmPidFF
import beaverlib.controls.PIDConstants
import beaverlib.controls.PidFF
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
import frc.robot.beaverlib.utils.sysID.BeaverSysIDMotor
import frc.robot.beaverlib.utils.sysID.BeaverSysIDRoutine
import frc.robot.engine.DashboardNumber
import frc.robot.engine.FFSendable

object Shooter : SubsystemBase() {
    private object Constants {
        const val MOTOR_1_ID = 16
        const val MOTOR_2_ID = 17

        val motor1PIDConstants = PIDConstants(0.47, 0.0, 0.0)

        val motor1FFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)

        val runningSpeed by DashboardNumber(0.0, "Shooter/Constants")
    }

    private val motor1 = SparkMax(Constants.MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless)
    private val motor2 = SparkMax(Constants.MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless)

    private val motor1Controller = PidFF(Constants.motor1PIDConstants, Constants.motor1FFConstants)

    init {
        val shooterConfig = SparkMaxConfig()
        shooterConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(20)
            .encoder
            .velocityConversionFactor(2.0)
        motor1.configure(
            shooterConfig.inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        motor2.configure(
            shooterConfig.inverted(false).follow(motor1),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        defaultCommand = stop()

        SmartDashboard.putData("Shooter/motor1/PID", motor1Controller.pid)
        SmartDashboard.putData("Shooter/motor1/FF", FFSendable(motor1Controller.feedforward))
    }
    val sysID : BeaverSysIDRoutine = BeaverSysIDRoutine(this, BeaverSysIDMotor("ShooterMotor", motor1))

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stop(): Command = runOnce {
        motor1.stopMotor()
        motor2.stopMotor()
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stabilize(): Command = run {
        motor1.set(motor1Controller.calculate(motor1.encoder.velocity))
    }

    fun waitSpeed(): Command = waitUntil {
        motor1Controller.atSetpoint())
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtSpeed(): Command =
        stabilize()
            .beforeStarting({
                motor1Controller.setpoint = Constants.runningSpeed
            })

    fun runAtPower(power: Double): Command = run {
        motor1.set(power)
    }

    object Hood : SubsystemBase() {
        private object Constants {
            const val MOTOR_ID = 18
            const val ENCODER_ID = 1

            val pidConstants = PIDConstants(0.0, 0.0, 0.0)
            val ffConstants = ArmFeedForwardConstants(0.0, 0.0, 0.0)

            val DOWN_POSITION = 0.0.radians
            val kS by DashboardNumber(0.1, "Shooter/Hood/Constants")
        }

        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        private val controller = ArmPidFF(Constants.pidConstants, Constants.ffConstants)

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            defaultCommand = stop() // todo doMoveDown()

            SmartDashboard.putData("Shooter/Hood/ArmPID", controller)
        }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stop(): Command = run {
            motor1.stopMotor()
            motor2.stopMotor()
        }

        fun doRunAtkS(): Command = run { motor.setVoltage(Constants.kS) }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun holdPosition(position: AngleUnit): Command =
            startRun({ controller.setpoint = position }) {
                motor.setVoltage(controller.calculate(absEncoder.get().rotations))
            }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun moveToPosition(position: AngleUnit): Command =
            holdPosition(position).withDeadline(waitUntil { controller.atSetpoint() })

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun moveDown() = holdPosition(Constants.DOWN_POSITION)
    }

    object Feeder : SubsystemBase() {
        private object Constants {
            const val MOTOR_ID = 15

            val pidConstants = PIDConstants(0.0, 0.0, 0.0)
            val ffConstants = SimpleMotorFeedForwardConstants(0.0, 0.0, 0.0)

            val runningSpeed by DashboardNumber(100.0, "Shooter/Feeder/Constants")
        }

        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
        private val controller = PidFF(Constants.pidConstants, Constants.ffConstants)

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            defaultCommand = stop()

            SmartDashboard.putData("Shooter/Feeder/PID", controller.pid)
            SmartDashboard.putData("Shooter/Feeder/FF", FFSendable(controller.feedforward))
        }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stop(): Command = runOnce { motor.stopMotor() }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtPower(power: Double): Command = run { motor.set(power) }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtSpeed(): Command =
            startRun({ controller.setpoint = Constants.runningSpeed }) {
                motor.set(controller.calculate(motor.encoder.velocity))
            }
    }
}
