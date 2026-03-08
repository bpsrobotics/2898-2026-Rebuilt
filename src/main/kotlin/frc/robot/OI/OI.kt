package frc.robot.OI

import beaverlib.utils.Units.Time
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.swerve.DriveManager
import frc.robot.engine.DashboardNumber
import frc.robot.subsystems.Drivetrain
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

/**
 * The Operating Interface object. This is where you put joystick, button, or keyboard inputs.
 *
 * A note about delegated properties, which are used in this object: A delegated property is where
 * getting (or setting) a field is outsourced to another object. Then, whenever you read the
 * property, it asks the object the property is delegated to for the value.
 */
object OI : SubsystemBase() {
    object Constants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val OPERATOR_CONTROLLER_PORT = 1
    }

    init {
        defaultCommand = rumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }

    val reverseDrive
        get() =
            if (
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) ==
                    DriverStation.Alliance.Red
            ) {
                1.0
            } else {
                -1.0
            }

    val driveManager = DriveManager()

    // private val hubDistance by DashboardNumber(2.0, "OI")
    val desiredHoodAngle by DashboardNumber(1.3, "OI", true)
    val desiredRPM by DashboardNumber(4500.0, "OI", true)
    val desiredShooterPower by DashboardNumber(1.0, "OI", true)

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * Trigger constructor with an arbitrary predicate, or via the named factories in [ ]'s
     * subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    fun configureBindings() {
        /** SysID */
        SmartDashboard.putData(
            "SysIdCommands/Drivetrain/DriveMotors",
            Drivetrain.sysIdDriveMotors(),
        )
        SmartDashboard.putData(
            "SysIdCommands/Drivetrain/AngleMotors",
            Drivetrain.sysIdAngleMotors(),
        )
        // SmartDashboard.putData("SysIdCommands/Shooter/Flywheel", Shooter.sysID.fullSysID())
        // SmartDashboard.putData("SysIdCommands/Shooter/HoodAngles", collectShooterAngles())
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
        deadzone: Double = DEADZONE_THRESHOLD,
        power: Double,
    ): Double {
        var output = 0.0

        if (deadzone != 0.0) {
            output = MathUtil.applyDeadband(input, deadzone)
        }

        if (power != 1.0) {
            // To keep the signage for output, we multiply by sign(output). This keeps negative
            // inputs
            // resulting in negative outputs.
            output = output.absoluteValue.pow(power) * sign(output)
        }

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Double = DEADZONE_THRESHOLD, power: Double) =
        process(this, deadzone, power)

    val driverController = CommandXboxController(Constants.DRIVER_CONTROLLER_PORT)
    val operatorController = CommandJoystick(Constants.OPERATOR_CONTROLLER_PORT)

    // Right joystick y-axis.  Controller mapping can be tricky, the best way is to use the driver
    // station to see what buttons and axis are being pressed.
    // Squared for better control on turn, cubed on throttle
    /**
     * Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right)
     */
    val translationX
        get() = process(driverController.leftX, power = 1.5)

    /**
     * Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up)
     */
    val translationY
        get() = process(driverController.leftY, power = 1.5)

    /**
     * Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1
     * (right)
     */
    val turn
        get() = process(driverController.rightX, power = 1.5)

    val rightTrigger
        get() = driverController.rightTriggerAxis

    val resetGyro: Trigger = driverController.rightBumper()

    val highHatForward: Trigger = operatorController.pov(0)
    val highHatBack: Trigger = operatorController.pov(180)
    val operatorTrigger: Trigger = operatorController.trigger()

    /** Rumbles the driver controller continuously until interrupted. */
    fun rumble(side: GenericHID.RumbleType, power: Double): Command =
        runEnd(
            { driverController.setRumble(side, power) },
            { driverController.setRumble(side, 0.0) },
        )

    /** Rumbles the driver controller until the given time has elapsed. */
    fun rumble(side: GenericHID.RumbleType, power: Double, time: Time): Command =
        rumble(side, power).withTimeout(time.asSeconds)
}
