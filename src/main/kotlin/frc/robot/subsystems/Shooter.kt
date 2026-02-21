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
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.beaverlib.utils.sysID.BeaverSysIDMotor
import frc.robot.beaverlib.utils.sysID.BeaverSysIDRoutine
import frc.robot.engine.DashboardBoolean
import frc.robot.engine.DashboardNumber
import frc.robot.engine.FFSendable
import frc.robot.subsystems.Shooter.Hood.absoluteEncoderOffset
import kotlin.math.PI

object Shooter : SubsystemBase() {
    private object Constants {
        const val MOTOR_1_ID = 16
        const val MOTOR_2_ID = 17

        val motor1PIDConstants = PIDConstants(0.47, 0.0, 0.0)

        val motor1FFConstants = SimpleMotorFeedForwardConstants(0.1, 0.19, 4.04)

        val runningSpeed by DashboardNumber(0.0, "Shooter/Constants")
    }

    private val motor = SparkMax(Constants.MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorFollower = SparkMax(Constants.MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless)

    private val motor1Controller = PidFF(Constants.motor1PIDConstants, Constants.motor1FFConstants)

    init {
        val shooterConfig = SparkMaxConfig()
        shooterConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(20)
            .inverted(true)
            .encoder
            .velocityConversionFactor(2.0)

        val followerConfig = SparkMaxConfig()
        followerConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(20)
            .follow(motor, true)
            .encoder
            .velocityConversionFactor(2.0)

        motor.configure(
            shooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        motorFollower.configure(
            followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        defaultCommand = stop()

        SmartDashboard.putData("Shooter/motor1/PID", motor1Controller)
        SmartDashboard.putData("Shooter/motor1/FF", FFSendable(motor1Controller.feedforward))
    }

    var motor1Current by DashboardNumber(0.0, "Shooter")
    var motor2Current by DashboardNumber(0.0, "Shooter")

    override fun periodic() {
        motor1Current = motor.outputCurrent
        motor2Current = motorFollower.outputCurrent
    }

    val sysID: BeaverSysIDRoutine =
        BeaverSysIDRoutine(this, BeaverSysIDMotor("ShooterMotor", motor))

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stop(): Command = runOnce {
        motor.stopMotor()
        motorFollower.stopMotor()
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stabilize(): Command = run { motor.set(motor1Controller.calculate(motor.encoder.velocity)) }

    fun waitSpeed(): Command = waitUntil { motor1Controller.atSetpoint() }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtSpeed(): Command =
        stabilize().beforeStarting({ motor1Controller.setpoint = Constants.runningSpeed })

    fun runAtPower(power: Double): Command = run { motor.set(power) }

    /**
     * Run the Intake at the given speed
     *
     * @param powerFun (-1, 1) the portion of max speed to run the motor at
     */
    fun runAtPower(powerFun: () -> Double): Command =
        runEnd(
            {
                println(powerFun())
                motor.set(powerFun())
            },
            { motor.stopMotor() },
        )

    object Hood : SubsystemBase() {
        object Constants {
            const val MOTOR_ID = 18
            const val ENCODER_ID = 1

            val pidConstants = PIDConstants(2.5, 0.2, 0.1)
            val ffConstants = ArmFeedForwardConstants(0.45, 0.1, 0.0)

            val DOWN_POSITION = 0.0.radians
            val TOP_POSITION =
                MathUtil.inputModulus(absEncoder.get() - absoluteEncoderOffset, -PI, PI).radians
            val kS by DashboardNumber(0.1, "Shooter/Hood/Constants")
        }

        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        private val controller = ArmPidFF(Constants.pidConstants, Constants.ffConstants)

        var absoluteEncoderOffset = 0.4060916601522915

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            absoluteEncoderOffset = absEncoder.get()

            defaultCommand = setDownAndReZero() // todo doMoveDown()

            SmartDashboard.putData("Shooter/Hood/ArmPID", controller)
        }

        val position: AngleUnit
            get() =
                MathUtil.inputModulus(absEncoder.get() - absoluteEncoderOffset, -PI, PI).rotations

        var hoodEncoderPosition by DashboardNumber(0.0, "Shooter/Hood")
        var hoodEncoderConnected by DashboardBoolean(false, "Shooter/Hood")
        var rawEncoderPosition by DashboardNumber(0.0, "Shooter/Hood")

        override fun periodic() {
            hoodEncoderPosition = position.asRadians
            hoodEncoderConnected = absEncoder.isConnected
            rawEncoderPosition = absEncoder.get()
        }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stop(): Command = run {
            motor.stopMotor()
            motorFollower.stopMotor()
        }

        fun doRunAtkS(): Command = run { motor.setVoltage(Constants.kS) }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun holdPosition(positionToHold: AngleUnit): Command =
            startRun({ controller.setpoint = positionToHold }) {
                motor.setVoltage(controller.calculate(position))
            }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun holdPosition(positionToHold: () -> AngleUnit): Command = run {
            controller.setpoint = positionToHold()
            motor.setVoltage(controller.calculate(position))
        }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stabilize(): Command = run { motor.setVoltage(controller.calculate(position)) }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun moveToPosition(positionToHold: AngleUnit): Command =
            holdPosition(positionToHold).withDeadline(waitUntil { controller.atSetpoint() })

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun moveToPosition(positionToHold: () -> AngleUnit): Command =
            holdPosition(positionToHold).withDeadline(waitUntil { controller.atSetpoint() })

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun moveDown() = holdPosition(Constants.DOWN_POSITION)

        fun setDownAndReZero() =
            run { motor.setVoltage(-2.0) }
                .until { motor.outputCurrent > 15 }
                .andThen(
                    WaitCommand(0.5),
                    runOnce { absoluteEncoderOffset = absEncoder.get() },
                    WaitCommand(1.0).repeatedly(),
                )
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
            motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(30)
            motor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

            defaultCommand = stop()

            SmartDashboard.putData("Shooter/Feeder/PidFF", controller)
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

        fun getJiggyWithIt(): Command =
            SequentialCommandGroup(
                    runAtPower(1.0).withTimeout(1.0),
                    runAtPower(-1.0).withTimeout(0.5),
                )
                .repeatedly()

        var motorCurrent by DashboardNumber(0.0, "Shooter/Indexer")

        override fun periodic() {
            motorCurrent = motor.outputCurrent
        }
    }
}
