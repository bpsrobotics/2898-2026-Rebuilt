package frc.robot.subsystems

import beaverlib.controls.ArmFeedForwardConstants
import beaverlib.controls.PIDConstants
import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.rotations
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.engine.DashboardNumber
import frc.robot.engine.HoodPIDFF
import frc.robot.engine.SparkWrapper

object Intake : SubsystemBase() {
    private object Constants {
        const val MOTOR_ID = 13
    }

    private val motor =
        SparkWrapper(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
            idleMode(SparkBaseConfig.IdleMode.kCoast)
            smartCurrentLimit(25)
            inverted(true)
        }

    init {
        // Intake motor initialization stuff
        defaultCommand = stop()

        SmartDashboard.putData("Intake/motor", motor)
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
    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun runAtPower(power: () -> Double): Command =
        runEnd({ motor.set(power()) }, { motor.stopMotor() })

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    /** Stops the Intake motor */
    fun stop(): Command = runOnce { motor.stopMotor() }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    object Pivot : SubsystemBase() {
        object Constants {
            const val MOTOR_ID = 14
            const val ENCODER_ID = 0
            const val ENCODER_OFFSET = -0.7716722942918074

            val pidConstants: PIDConstants = PIDConstants(3.5, 0.05, 0.0)
            val armFFConstants = ArmFeedForwardConstants(0.75, 0.3, 0.0)
            val STOWED_POSITION = 1.722657031745036.radians
            val EXTENDED_POSITION = 0.0.radians
            val FEEDER_POSITION = 0.3284585466925222.radians
        }

        // Initializing brushless motor with SparkMAX motor controller
        private val motor =
            SparkWrapper(Constants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless) {
                idleMode(SparkBaseConfig.IdleMode.kCoast)
                smartCurrentLimit(35)
            }

        // Use encoder values for PID tuning
        private val absEncoder = DutyCycleEncoder(Constants.ENCODER_ID)

        // PID controller class for pivot subsystem
        private val controller = HoodPIDFF(Constants.pidConstants, Constants.armFFConstants)

        val position
            get() =
                MathUtil.inputModulus(absEncoder.get() + Constants.ENCODER_OFFSET, -0.5, 0.5)
                    .rotations

        init {
            val motorConfig = SparkMaxConfig()

            SmartDashboard.putData("Intake/Pivot/ArmPidFF", controller)
            // SmartDashboard.putData("Intake/Pivot/motor", Intake.motor)
            controller.setpoint = position
            // Stabilize the wrist if nothing else is happening
            defaultCommand = stabilize()
        }

        var pivotEncoderPosition by DashboardNumber(0.0, "Intake/Pivot/Position")
        var rawEncoderPosition by DashboardNumber(0.0, "Intake/Pivot")

        override fun periodic() {
            pivotEncoderPosition = position.asRadians
            rawEncoderPosition = absEncoder.get()
        }

        /** Stops the wrist */
        fun stop(): Command = runOnce { motor.stopMotor() }

        /** Holds the wrist at the last set position */
        fun stabilize(): Command = run { motor.setVoltage(controller.calculate(position)) }

        fun setSetpoint(newSetpoint: AngleUnit): Command =
            InstantCommand({ controller.setpoint = newSetpoint }, this)

        /** Sets the wrist to target position, and ends once the PID is at the setpoint */
        fun runToPosition(targetPosition: AngleUnit): Command =
            run {
                    controller.setpoint = targetPosition
                    motor.setVoltage(controller.calculate(position))
                }
                .until { controller.atSetpoint() }

        fun runAtPower(power: Double): Command = run { motor.set(power) }

        fun runAtkS(): Command = run { motor.setVoltage(controller.kS) }

        fun getJiggyWithIt(): Command =
            runToPosition(Constants.FEEDER_POSITION)
                .withTimeout(1.5)
                .andThen(stow().withTimeout(1.5))
                .repeatedly()

        fun extend() = runToPosition(Constants.EXTENDED_POSITION)

        fun stow() = runToPosition(Constants.STOWED_POSITION)
    }
}
