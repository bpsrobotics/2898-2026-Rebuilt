package frc.robot.subsystems

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.ArmPidFF
import beaverlib.controls.PIDConstants
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.degrees
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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.engine.DashboardNumber

object Intake : SubsystemBase() {
    private object Constants {
        const val MOTOR_ID = 13
    }

    private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    init {
        // Intake motor initialization stuff
        val motorConfig = SparkMaxConfig()
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(20).inverted(true)
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        defaultCommand = stop()
    }

    override fun periodic() {
        SmartDashboard.putNumber("Intake/motorCurrent", motor.outputCurrent)
        SmartDashboard.putNumber("Intake motor temperature:", motor.motorTemperature)
    }

    /**
     * Run the Intake at the given speed
     *
     * @param power (-1, 1) the portion of max speed to run the motor at
     */
    fun runAtPower(power: Double): Command = runEnd({ motor.set(power) }, { motor.stopMotor() })

    /**
     * Run the Intake at the given speed
     *
     * @param power (-1, 1) the portion of max speed to run the motor at
     */
    fun runAtPower(power: () -> Double): Command =
        runEnd({ motor.set(power()) }, { motor.stopMotor() })

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    /** Stops the Intake motor */
    fun stop(): Command = runOnce { motor.stopMotor() }

    object Pivot : SubsystemBase() {
        private object Constants {
            const val MOTOR_ID = 14
            const val ENCODER_ID = 0
            const val ENCODER_OFFSET = 0.48418893710472344

            val pidConstants: PIDConstants = PIDConstants(0.67, 0.0, 0.0)
            val armFFConstants = ArmFeedForwardConstants(0.0, 0.98, 0.0)
            val STOWED_POSITION = 0.degrees
            val EXTENDED_POSITION = 0.803673142114475.radians
        }

        var encoderPosition by DashboardNumber(0.0, "Intake/Pivot")
        var motorCurrent by DashboardNumber(0.0, "Intake/Pivot")

        // Initializing brushless motor with SparkMAX motor controller
        private val motor = SparkMax(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

        // Use encoder values for PID tuning
        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        // PID controller class for pivot subsystem
        private val controller = ArmPidFF(Constants.pidConstants, Constants.armFFConstants)

        val position
            get() =
                MathUtil.inputModulus(absEncoder.get() + Constants.ENCODER_OFFSET, -0.5, 0.5)
                    .rotations

        override fun periodic() {
            encoderPosition = position.asRadians
            motorCurrent = motor.outputCurrent
        }

        init {
            val motorConfig = SparkMaxConfig()
            motorConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                // TODO: Tune current limit but Recalc says we need at least 30A
                .smartCurrentLimit(30)
            motor.configure(
                motorConfig,
                // The reset mote and persist mode have to do with maintaining
                // settings after a power cycle.
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            SmartDashboard.putData("Intake/Pivot/ArmPidFF", controller)

            // Stabilize the wrist if nothing else is happening
            defaultCommand = stop()
        }

        /** Stops the wrist */
        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stop(): Command = runOnce { motor.stopMotor() }

        /** Holds the wrist at the last set position */
        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun stabilize(): Command = run { motor.setVoltage(controller.calculate(position)) }

        /** Sets the wrist to target position, and ends once the PID is at the setpoint */
        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runToPosition(targetPosition: AngleUnit): Command =
            stabilize()
                .beforeStarting(InstantCommand({ controller.setpoint = targetPosition }))
                .until { controller.atSetpoint() }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtPower(power: Double): Command = run { motor.set(power) }

        @Suppress("MemberVisibilityCanBePrivate", "unused")
        fun runAtkS(): Command = run { motor.setVoltage(controller.feedforward.ks) }

        fun getJiggyWithIt() =
            extend().withTimeout(1.0).andThen(stow().withTimeout(1.0)).repeatedly()

        fun extend() = runToPosition(Constants.EXTENDED_POSITION)

        fun stow() = runToPosition(Constants.STOWED_POSITION)
    }
}
