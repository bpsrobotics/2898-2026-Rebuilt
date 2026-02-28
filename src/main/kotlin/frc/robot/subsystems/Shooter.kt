package frc.robot.subsystems

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.ArmPidFF
import beaverlib.controls.PIDConstants
import beaverlib.controls.PidFF
import beaverlib.controls.SimpleMotorFeedForwardConstants
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.asRPM
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.rotations
import com.revrobotics.spark.SparkLowLevel
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
import frc.engine.utils.Polynomial
import frc.robot.engine.DashboardBoolean
import frc.robot.engine.DashboardNumber
import frc.robot.engine.SparkWrapper
import frc.robot.subsystems.Shooter.Constants.MOTOR_1_ID
import frc.robot.subsystems.Shooter.Hood.absoluteEncoderOffset
import kotlin.math.PI

object Shooter : SubsystemBase() {
    private object Constants {
        const val MOTOR_1_ID = 16
        const val MOTOR_2_ID = 17

        val motor1PIDConstants = PIDConstants(0.005, 0.00032, 0.0012)

        val motor1FFConstants = SimpleMotorFeedForwardConstants(0.564, 0.00098075996, 0.0)

        val runningSpeed by DashboardNumber(0.0, "Shooter/Constants")
    }

    private val motor =
        SparkWrapper(Constants.MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless) {
            idleMode(SparkBaseConfig.IdleMode.kCoast)
            smartCurrentLimit(20)
            inverted(true)
            encoder.velocityConversionFactor(2.0)
        }
    private val motorFollower =
        SparkWrapper(Constants.MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless) {
            apply(motor.config)
            follow(MOTOR_1_ID, true)
        }

    private val motor1Controller = PidFF(Constants.motor1PIDConstants, Constants.motor1FFConstants)

    init {
        defaultCommand = stop()

        SmartDashboard.putData("Shooter/motor/PID", motor1Controller)
    }

    var motor1Velocity by DashboardNumber(0.0, "Shooter")
    var motor1DesiredPower by DashboardNumber(0.0, "Shooter")

    var motor1Current by DashboardNumber(0.0, "Shooter")
    var motor2Current by DashboardNumber(0.0, "Shooter")

    override fun periodic() {}

    /*val sysID: BeaverSysIDRoutine =
    BeaverSysIDRoutine(this, BeaverSysIDMotor("ShooterMotor", motor))*/
    var motorVoltage: Double by DashboardNumber(0.0, "Shooter")

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stop(): Command = runOnce {
        motor.stopMotor()
        motorFollower.stopMotor()
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stabilize(): Command = run {
        motorVoltage = motor1Controller.calculate(motor.velocity.asRPM)
        motor.set(motor1Controller.calculate(motor.velocity.asRPM).clamp(0.0, 12.0))
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun waitSpeed(): Command = waitUntil { motor1Controller.atSetpoint() }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtSpeed(): Command =
        stabilize().beforeStarting({ motor1Controller.setpoint = Constants.runningSpeed })

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtSpeed(speedLambda: () -> AngularVelocity): Command = run {
        motor1Controller.setpoint = speedLambda().asRPM
        motorVoltage = motor1Controller.calculate(motor.velocity.asRPM)
        motor.setVoltage(motor1Controller.calculate(motor.velocity.asRPM).clamp(0.0, 12.0))
    }

    val desiredSpeed: Double by DashboardNumber(0.0, "Shooter")

    fun runAtDashboardPower(): Command = run { motor.set(motor1DesiredPower) }

    fun runAtPower(power: Double): Command = run { motor.set(power) }

    /**
     * Run the Intake at the given speed
     *
     * @param powerFun (-1, 1) the portion of max speed to run the motor at
     */
    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtPower(powerFun: () -> Double): Command =
        runEnd({ motor.set(powerFun()) }, { motor.stopMotor() })

    object Hood : SubsystemBase() {
        object Constants {
            const val MOTOR_ID = 18
            const val ENCODER_ID = 1

            val pidConstants = PIDConstants(1.5, 0.4, 0.4)
            val ffConstants = ArmFeedForwardConstants(0.25, 0.1, 0.0)

            val DOWN_POSITION = 0.0.radians
            val TOP_POSITION = 2.7.radians
        }

        private val motor =
            SparkWrapper(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
                smartCurrentLimit(30)
            }

        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        private val controller = ArmPidFF(Constants.pidConstants, Constants.ffConstants)

        var absoluteEncoderOffset = 0.4060916601522915

        init {
            val motorConfig = SparkMaxConfig()

            absoluteEncoderOffset = absEncoder.get()

            defaultCommand = setDownAndReZero()
            controller.pid.setTolerance(0.02)

            SmartDashboard.putData("Shooter/Hood/ArmPID", controller)
            // SmartDashboard.putData("Shooter/Hood/motor", motor)
        }

        val position: AngleUnit
            get() =
                MathUtil.inputModulus(absEncoder.get() - absoluteEncoderOffset, -PI, PI).rotations

        var hoodEncoderPosition by DashboardNumber(0.0, "Shooter/Hood")
        var hoodEncoderConnected by DashboardBoolean(false, "Shooter/Hood")
        var rawEncoderPosition by DashboardNumber(0.0, "Shooter/Hood")
        var motorVoltage by DashboardNumber(0.0, "Shooter/Hood")

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

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun holdPosition(positionToHold: AngleUnit): Command =
            startRun({ controller.setpoint = positionToHold }) {
                motorVoltage = controller.calculate(position)
                motor.setVoltage(controller.calculate(position))
            }

        val desiredVoltage: Double by DashboardNumber(0.0, "Shooter/Hood")

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtDashboardVoltage(): Command = run {
            motorVoltage = desiredVoltage
            motor.setVoltage(desiredVoltage)
        }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun holdPosition(positionToHold: () -> AngleUnit): Command = run {
            controller.setpoint = positionToHold()
            motorVoltage = controller.calculate(position)
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

        fun setDownAndReZero(): Command =
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

        private val motor =
            SparkWrapper(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
                smartCurrentLimit(30)
            }
        private val controller = PidFF(Constants.pidConstants, Constants.ffConstants)

        init {
            defaultCommand = stop()

            SmartDashboard.putData("Shooter/Feeder/PidFF", controller)
            // SmartDashboard.putData("Shooter/Feeder/motor", motor)
        }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stop(): Command = runOnce { motor.stopMotor() }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtPower(power: Double): Command = run { motor.set(power) }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtSpeed(): Command =
            startRun({ controller.setpoint = Constants.runningSpeed }) {
                motor.set(controller.calculate(motor.velocity.asRPM))
            }

        fun getJiggyWithIt(): Command =
            SequentialCommandGroup(
                    runAtPower(1.0).withTimeout(1.0),
                    runAtPower(-1.0).withTimeout(0.5),
                )
                .repeatedly()
    }
}
