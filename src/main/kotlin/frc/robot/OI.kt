package frc.robot

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.OI.process
import frc.robot.commands.DoOpenloopIntake
import frc.robot.commands.DoOutakeFullRobot
import frc.robot.commands.DoShoot
import frc.robot.commands.DoShootIntake
import frc.robot.commands.OI.NavXReset
import frc.robot.commands.OI.Rumble
import frc.robot.commands.autos.AutoShootCarrots
import kotlin.math.pow
import kotlin.math.sign

/**
 * The Operating Interface object. This is where you put joystick, button, or keyboard inputs.
 *
 * A note about delegated properties, which are used in this object: A delegated property is where
 * getting (or setting) a field is outsourced to another object. Then, whenever you read the
 * property, it asks the object the property is delegated to for the value.
 */
@Suppress("unused")
object OI : SubsystemBase() {
    object Constants {
        const val DriverControllerPort = 0
        const val OperatorControllerPort = 1
    }

    init {
        defaultCommand = Rumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }

    val navXResetCommand: NavXReset = NavXReset()

    // val followTagCommand = FollowApriltagGood(18)

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * Trigger constructor with an arbitrary predicate, or via the named factories in [ ]'s
     * subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight][CommandJoystick].
     */
    fun configureBindings() {
        resetGyro.whileTrue(navXResetCommand)
        // If high hat is moved towards the player, and NOT shooting, run intake
        highHatBack.and(operatorTrigger.negate()).whileTrue(DoOpenloopIntake())

        // If high hat is moved away from the player, run all subsystems in reverse to dislodge
        // stuck carrots
        highHatForward.whileTrue(DoOutakeFullRobot()) // Outtake

        SmartDashboard.putNumber("Shooter/DesiredShooterRPM", 3500.0)

        // If operator trigger is pressed, and not intaking, run the shoot command
        operatorTrigger
            .and(highHatBack.negate())
            .whileTrue(
                DoShoot({ SmartDashboard.getNumber("Shooter/DesiredShooterRPM", 3500.0).RPM })
            )

        // If operator trigger is pressed, and ALSO intaking, run the shoot and intake command
        operatorTrigger
            .and(highHatBack)
            .whileTrue(
                DoShootIntake({ SmartDashboard.getNumber("Shooter/DesiredShooterRPM", 3500.0).RPM })
            )
        driverController.leftTrigger().whileTrue(AutoShootCarrots)

        //        driverController.x().whileTrue(Shooter.routine.fullSysID())

        //        driverController.y().whileTrue(sysIdDriveMotor())
        //        driverController.a().whileTrue(
        //            sysIdAngleMotorCommand())

    }

    /**
     * Threshold below which [process] will return 0. 0.1 historically used, but optimal value
     * unknown.
     */
    private const val DEADZONE_THRESHOLD = 0.1

    /**
     * Utility function for controller axis, optional deadzone and square/cube for extra fine-grain
     * control
     */
    private fun process(
        input: Double,
        deadzone: Boolean = false,
        square: Boolean = false,
        cube: Boolean = false,
    ): Double {
        var output = 0.0

        if (deadzone) {
            output = MathUtil.applyDeadband(input, DEADZONE_THRESHOLD)
        }

        if (square) {
            // To keep the signage for output, we multiply by sign(output). This keeps negative
            // inputs
            // resulting in negative outputs.
            output = output.pow(2) * sign(output)
        }

        if (cube) {
            // Because cubing is an odd number of multiplications, we don't need to multiply by
            // sign(output) here.
            output = output.pow(3)
        }

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Boolean = false, square: Boolean = false, cube: Boolean = false) =
        process(this, deadzone, square, cube)

    val driverController = CommandXboxController(Constants.DriverControllerPort)
    private val operatorController = CommandJoystick(Constants.OperatorControllerPort)

    // Right joystick y-axis.  Controller mapping can be tricky, the best way is to use the driver
    // station to see what buttons and axis are being pressed.
    // Squared for better control on turn, cubed on throttle
    /**
     * Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right)
     */
    val translationX
        get() = process(driverController.leftX, deadzone = true, square = false)

    /**
     * Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up)
     */
    val translationY
        get() = process(driverController.leftY, deadzone = true, square = false)

    /**
     * Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1
     * (right)
     */
    val turnX
        get() = process(driverController.rightX, deadzone = true, square = false)

    /**
     * Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up)
     */
    val turnY
        get() = process(driverController.rightY, deadzone = true, square = false)

    val leftTrigger
        get() = driverController.leftTriggerAxis

    val rightTrigger
        get() = driverController.rightTriggerAxis

    val resetGyro: Trigger = driverController.rightBumper()
    val followTag: Trigger = driverController.leftBumper()
    val sysidFQ: Trigger = driverController.x()
    val sysidBQ: Trigger = driverController.y()
    val sysidFD: Trigger = driverController.b()
    val sysidBD: Trigger = driverController.a()

    val highHatForward: Trigger = operatorController.pov(0)
    val highHatBack: Trigger = operatorController.pov(180)
    val operatorTrigger: Trigger = operatorController.trigger()

    //    val hatVector get() = when (operatorController.pov) {
    //        0 -> Vector2(0.0,1.0)
    //        90 -> Vector2(1.0,0.0)
    //        180 -> Vector2(0.0,-1.0)
    //        270 -> Vector2(-1.0,0.0)
    //        else -> Vector2.zero()
    //    }

    val intakeSpeed
        get() = operatorController.throttle

    enum class Direction {
        LEFT,
        RIGHT,
        UP,
        DOWN,
        UPLEFT,
        UPRIGHT,
        DOWNLEFT,
        DOWNRIGHT,
        INACTIVE;

        fun mirrored() =
            when (this) {
                LEFT -> RIGHT
                RIGHT -> LEFT
                else -> this
            }

        fun toVector() =
            when (this) {
                LEFT -> Vector2(-1.0, 0.0)
                RIGHT -> Vector2(1.0, 0.0)
                UP -> Vector2(0.0, 1.0)
                DOWN -> Vector2(0.0, -1.0)
                INACTIVE -> Vector2.zero()
                UPLEFT -> Vector2(-1.0, 1.0)
                UPRIGHT -> Vector2(1.0, 1.0)
                DOWNLEFT -> Vector2(-1.0, -1.0)
                DOWNRIGHT -> Vector2(1.0, -1.0)
            }
    }
}
