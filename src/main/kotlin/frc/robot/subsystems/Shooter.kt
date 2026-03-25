package frc.robot.subsystems

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.PIDConstants
import beaverlib.controls.PidFF
import beaverlib.controls.SimpleMotorFeedForwardConstants
import beaverlib.utils.MovingAverage
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.asRPM
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.rotations
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.engine.utils.Polynomial
import frc.robot.engine.DashboardNumber
import frc.robot.engine.HoodPIDFF
import frc.robot.engine.SparkWrapper
import kotlin.math.PI

@Suppress("MemberVisibilityCanBePrivate", "unused")
object Shooter : SubsystemBase() {
    private object Constants {
        const val MOTOR_1_ID = 16
        const val MOTOR_2_ID = 17

        val motor1PIDConstants = PIDConstants(0.005, 0.0, 0.001)

        val motor1FFConstants = SimpleMotorFeedForwardConstants(0.25, 0.00105, 0.0)

        val runningSpeed by DashboardNumber(0.0, "Shooter/Constants")
    }

    val atSpeed
        get() = motor1Controller.atSetpoint()

    private val motor =
        SparkWrapper(Constants.MOTOR_1_ID, SparkLowLevel.MotorType.kBrushless) {
            idleMode(SparkBaseConfig.IdleMode.kCoast)
            smartCurrentLimit(40)
            inverted(true)
            encoder.velocityConversionFactor(2.0)
        }
    private val motorFollower =
        SparkWrapper(Constants.MOTOR_2_ID, SparkLowLevel.MotorType.kBrushless) {
            apply(motor.config)
            follow(Constants.MOTOR_1_ID, true)
        }

    private val motor1Controller = PidFF(Constants.motor1PIDConstants, Constants.motor1FFConstants)

    init {
        defaultCommand = stop()

        SmartDashboard.putData("Shooter/motor/PID", motor1Controller)
    }


    override fun periodic() {}

    /*val sysID: BeaverSysIDRoutine =
    BeaverSysIDRoutine(this, BeaverSysIDMotor("ShooterMotor", motor))*/
    var motorVoltage: Double by DashboardNumber(0.0, "Shooter")

    fun stop(): Command = runOnce {
        motor.stopMotor()
        motorFollower.stopMotor()
    }

    fun stabilize(): Command = run {
        motorVoltage = motor1Controller.calculate(motor.velocity.asRPM)
        motor.set(motor1Controller.calculate(motor.velocity.asRPM).clamp(0.0, 12.0))
    }

    fun waitSpeed(): Command = waitUntil { motor1Controller.atSetpoint() }

    fun runAtSpeed(): Command =
        stabilize().beforeStarting({ motor1Controller.setpoint = Constants.runningSpeed })

    fun runAtSpeed(speedLambda: () -> AngularVelocity): Command = run {
        motor1Controller.setpoint = speedLambda().asRPM
        motorVoltage = motor1Controller.calculate(motor.velocity.asRPM)
        motor.setVoltage(motor1Controller.calculate(motor.velocity.asRPM).clamp(0.0, 12.0))
    }

    val desiredSpeed: Double by DashboardNumber(0.0, "Shooter")

    fun runAtPower(power: Double): Command = run { motor.set(power) }

    /**
     * Run the Intake at the given speed
     *
     * @param powerFun (-1, 1) the portion of max speed to run the motor at
     */
    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtPower(powerFun: () -> Double): Command =
        runEnd({ motor.setVoltage(powerFun()) }, { motor.stopMotor() })

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    object Hood : SubsystemBase() {
        object Constants {
            const val MOTOR_ID = 18
            const val ENCODER_ID = 1

            val pidConstants = PIDConstants(1.0, 0.0, 0.0)
            val ffConstants = ArmFeedForwardConstants(0.2, 0.20, 0.0)

            val DOWN_POSITION = 0.0.radians
            val TOP_POSITION = 2.7.radians

            val kinematics = Polynomial(0.196001, 1.76799, 5.48396, -4.12772)
        }

        private val motor =
            SparkWrapper(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
                smartCurrentLimit(30)
            }

        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        private val controller =
            HoodPIDFF(Constants.pidConstants, Constants.ffConstants, (22.degrees * 133.0) / 24.0)

        private var absoluteEncoderOffset = 0.4060916601522915

        init {
            absoluteEncoderOffset = absEncoder.get()

            defaultCommand = setDownAndReZero()
            controller.pid.setTolerance(0.04)

            SmartDashboard.putData("Shooter/Hood/ArmPID", controller)
        }

        val position: AngleUnit
            get() =
                MathUtil.inputModulus(absEncoder.get() - absoluteEncoderOffset, -PI, PI).rotations

        var setpoint
            get() = controller.setpoint
            set(v) {
                controller.setpoint = v
            }

        val atSetpoint
            get() = controller.atSetpoint()

        var hoodEncoderPosition by DashboardNumber(0.0, "Shooter/Hood")
        var rawEncoderPosition by DashboardNumber(0.0, "Shooter/Hood")
        var motorVoltage by DashboardNumber(0.0, "Shooter/Hood")
        var yellowBabber by DashboardNumber(0.0, "Shooter/Hood")

        override fun periodic() {
            hoodEncoderPosition = position.asRadians
            rawEncoderPosition = absEncoder.get()
            yellowBabber = currentAverage.average
        }

        fun applyController(setpoint: AngleUnit? = null) {
            if (setpoint != null)
                controller.setpoint =
                    setpoint.asRadians.clamp(0.0, Constants.TOP_POSITION.asRadians).radians
            motorVoltage = controller.calculate(position)
            motor.setVoltage(controller.calculate(position))
        }

        fun stop(): Command = run { motor.stopMotor() }

        fun runAtVoltage(desiredVoltage: Double): Command = run {
            motorVoltage = desiredVoltage
            motor.setVoltage(desiredVoltage)
        }

        fun holdPosition(positionToHold: AngleUnit): Command =
            startRun({ controller.setpoint = positionToHold }) { applyController() }

        fun holdPosition(positionToHold: () -> AngleUnit): Command = run {
            applyController(positionToHold())
        }

        fun stabilize(): Command = run { applyController() }

        fun moveToPosition(positionToHold: AngleUnit): Command =
            holdPosition(positionToHold).withDeadline(waitUntil { controller.atSetpoint() })

        fun moveToPosition(positionToHold: () -> AngleUnit): Command =
            holdPosition(positionToHold).withDeadline(waitUntil { controller.atSetpoint() })

        fun moveDown() = holdPosition(Constants.DOWN_POSITION)

        val currentAverage: MovingAverage = MovingAverage(5)
        fun resetCommand(): Command = startRun({ currentAverage.clear() }) {
            motor.setVoltage(-2.0)
            currentAverage.add(motor.outputCurrent)
        }
            .until { currentAverage.average > 20 }

        fun setDownAndReZero(): Command =
            startRun({ currentAverage.clear() }) {
                    motor.setVoltage(-2.0)
                    currentAverage.add(motor.outputCurrent)
                }
                .until { currentAverage.average > 20 }
                .andThen(
                    WaitCommand(0.5),
                    runOnce { absoluteEncoderOffset = absEncoder.get() },
                    stop().repeatedly(),
                )
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    object Feeder : SubsystemBase() {
        private object Constants {
            const val TOP_MOTOR_ID = 15
            const val BOTTOM_MOTOR_ID = 19

            val pidConstants = PIDConstants(0.0, 0.0, 0.0)
            val ffConstants = SimpleMotorFeedForwardConstants(0.0, 0.0, 0.0)

            val runningSpeed by DashboardNumber(100.0, "Shooter/Feeder/Constants")
        }

        private val topMotor =
            SparkWrapper(Constants.TOP_MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
                smartCurrentLimit(20)
            }
        private val bottomMotor =
            SparkWrapper(Constants.BOTTOM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
                smartCurrentLimit(20)
                inverted(true)
            }
        private val controller = PidFF(Constants.pidConstants, Constants.ffConstants)

        init {
            defaultCommand = stop()

            SmartDashboard.putData("Shooter/Feeder/PidFF", controller)
        }
        
        

        fun stop(): Command = runOnce {
            topMotor.stopMotor()
            bottomMotor.stopMotor()
        }

        fun runAtPower(power: Double): Command = run {
            topMotor.set(power)
            bottomMotor.set(power)
        }

        fun runAtSpeed(): Command =
            startRun({ controller.setpoint = Constants.runningSpeed }) {
                topMotor.set(controller.calculate(topMotor.velocity.asRPM))
                bottomMotor.set(controller.calculate(bottomMotor.velocity.asRPM))
            }
/* 
        fun getJiggyWithIt(power: Double): Command =
            SequentialCommandGroup(
                    run {
                            topMotor.set(power)
                            bottomMotor.set(power)
                        }
                        .withTimeout(0.66),
                    run {
                            topMotor.set(power)
                            bottomMotor.set(-power)
                        }
                        .withTimeout(0.33),
                )
                .repeatedly() */

        override fun periodic {
            var isStalled = stallDebouncer.calculate(topMotor.current >= 20 || bottomMotor.current >= 20)
        }
        val stallDebouncer = Debouncer(0.25, DebounceType.RISING) //True when current is above stall for > 1 second
        fun deJam(power: Double): Command =
            run {
                topMotor.set(power)
                bottomMotor.set(-power)
            }
            .withTimeout(0.33)

        fun feedBalls(power: Double): Command =
            run {
                topMotor.set(power)
                bottomMotor.set(power)
            }
        fun getJiggyWithIt(power: Double): Command =
            feedBalls(power).until(isStalled).andThen(deJam(power)).repeatedly()
    }
}
