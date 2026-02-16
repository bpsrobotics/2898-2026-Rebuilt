package frc.robot

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.Units.Time
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.OI.process
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.commands.vision.doCirclePoint
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.HedgieHelmet.trenchDriveTrigger
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.VisionTurningHandler
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
@Suppress("unused")
object OI : SubsystemBase() {
    object Constants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val OPERATOR_CONTROLLER_PORT = 1
    }

    init {
        defaultCommand = rumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }

    private val isEnabled = Trigger { DriverStation.isEnabled() }
    val reverseDrive =
        if (
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) ==
                DriverStation.Alliance.Red
        ) {
            -1.0
        } else {
            1.0
        }
    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { translationY * reverseDrive },
            { translationX * reverseDrive },
            { -turnX },
            { true },
            { rightTrigger },
        )
    val teleopDriveVisionTurn: TeleopDriveCommand =
        TeleopDriveCommand(
            { translationY * reverseDrive },
            { translationX * reverseDrive },
            VisionTurningHandler::rotationSpeed,
            { true },
            { rightTrigger },
            VisionTurningHandler::initialize,
        )

    // val followTagCommand = FollowApriltagGood(18)

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * Trigger constructor with an arbitrary predicate, or via the named factories in [ ]'s
     * subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight][CommandJoystick].
     */
    fun configureBindings() {
        // Drivetrain
        resetGyro
            .debounce(0.15)
            .onTrue(
                InstantCommand({ Drivetrain.zeroGyro() }, Drivetrain)
                    .andThen(rumble(GenericHID.RumbleType.kRightRumble, 0.25, 0.2.seconds))
            )
        Drivetrain.defaultCommand = teleopDrive
        driverController.a().whileTrue(teleopDriveVisionTurn)
        trenchDriveTrigger.onTrue(rumble(GenericHID.RumbleType.kBothRumble, 0.5))
        driverController
            .leftTrigger()
            .whileTrue(
                doCirclePoint(FieldMapREBUILTWelded.teamHub.center, 2.meters) {
                    translationX.degrees
                }
            )

        // Shooter
        isEnabled.whileTrue(Shooter.runSpeed())
        operatorController
            .trigger()
            .whileTrue(SequentialCommandGroup(Shooter.waitSpeed(), Shooter.Feeder.runSpeed()))
        driverController
            .a()
            .and(trenchDriveTrigger.negate())
            .whileTrue(Shooter.Hood.toPosition(0.1))

        // Intake
        highHatBack.whileTrue(Intake.runAtPower(0.05))
        highHatForward.whileTrue(Intake.runAtPower(-0.05))
        operatorController.axisGreaterThan(0, 0.5).onTrue(Intake.Pivot.stow())
        operatorController.axisLessThan(0, -0.5).onTrue(Intake.Pivot.extend())

        // SysID
        SmartDashboard.putData("SysIdCommands/Drivetrain/DriveMotors", Drivetrain.sysIdDriveMotor())
        SmartDashboard.putData(
            "SysIdCommands/Drivetrain/TurnMotors",
            Drivetrain.sysIdAngleMotorCommand(),
        )
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

    private val driverController = CommandXboxController(Constants.DRIVER_CONTROLLER_PORT)
    private val operatorController = CommandJoystick(Constants.OPERATOR_CONTROLLER_PORT)

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
    val turnX
        get() = process(driverController.rightX, power = 1.5)

    /**
     * Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up)
     */
    val turnY
        get() = process(driverController.rightY, power = 1.5)

    val leftTrigger
        get() = driverController.leftTriggerAxis

    val rightTrigger
        get() = driverController.rightTriggerAxis

    val resetGyro: Trigger = driverController.rightBumper()
    val followTag: Trigger = driverController.leftBumper()

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
        UP_LEFT,
        UP_RIGHT,
        DOWN_LEFT,
        DOWN_RIGHT,
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
                UP_LEFT -> Vector2(-1.0, 1.0)
                UP_RIGHT -> Vector2(1.0, 1.0)
                DOWN_LEFT -> Vector2(-1.0, -1.0)
                DOWN_RIGHT -> Vector2(1.0, -1.0)
            }
    }

    /** Rumbles the driver controller continuously until interrupted. */
    fun rumble(side: GenericHID.RumbleType, power: Double): Command = run {
        driverController.setRumble(side, power)
    }

    /** Rumbles the driver controller until the given time has elapsed. */
    fun rumble(side: GenericHID.RumbleType, power: Double, time: Time): Command =
        rumble(side, power).withTimeout(time.asSeconds)
}
