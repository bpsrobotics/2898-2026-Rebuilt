package frc.robot

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.Units.Time
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.OI.process
import frc.robot.commands.swerve.DriveManager
import frc.robot.commands.swerve.HubAlign
import frc.robot.commands.swerve.HubDistanceController
import frc.robot.commands.swerve.LockDrive
import frc.robot.commands.swerve.TeleopDrive
import frc.robot.engine.DashboardNumber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.HedgieHelmet.trenchDriveTrigger
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
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

    private val reverseDrive
        get() =
            if (
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) ==
                    DriverStation.Alliance.Red
            ) {
                1.0
            } else {
                -1.0
            }

    private val driveManager = DriveManager()

    // private val hubDistance by DashboardNumber(2.0, "OI")
    val desiredHoodAngle by DashboardNumber(1.3, "OI", true)
    val desiredRPM by DashboardNumber(4000.0, "OI", true)
    val desiredShooterPower by DashboardNumber(1.0, "OI", true)

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * Trigger constructor with an arbitrary predicate, or via the named factories in [ ]'s
     * subclasses for [ ]/[ PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight][CommandJoystick].
     */
    fun configureBindings() {
        /** Drivetrain */
        resetGyro
            .debounce(0.5)
            .onTrue(
                InstantCommand({ Drivetrain.zeroGyro() }, Drivetrain)
                    .andThen(rumble(GenericHID.RumbleType.kLeftRumble, 0.25, 0.2.seconds))
            )

        Drivetrain.defaultCommand =
            driveManager.alongWith(
                driveManager.defineDriver(
                    TeleopDrive(
                        { translationY * reverseDrive },
                        { translationX * reverseDrive },
                        { -turn },
                        { rightTrigger },
                    )
                )
            )
        //        driverController
        //            .a()
        //            .or(driverController.y())
        driverController
            .a()
            .or(driverController.y())
            .whileTrue(driveManager.defineDriver(HubAlign()))
            .and(trenchDriveTrigger.negate())
            .whileTrue(
                Shooter.Hood.holdPosition {
                    Shooter.Hood.Constants.kinematics
                        .calculate(
                            Drivetrain.pose.vector2.distance(FieldMapREBUILTWelded.teamHub.center)
                        )
                        .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
                        .radians
                }
            )

        driverController.b().debounce(0.2).whileTrue(driveManager.defineDriver(TrenchAlign()))

        driverController.x().debounce(0.2).whileTrue(driveManager.defineDriver(LockDrive()))
        driverController
            .y()
            .whileTrue(
                driveManager.defineDriver(
                    HubDistanceController(
                        desiredDistance = { 2.0.meters },
                        moveAround = { translationX },
                    )
                )
            )
        //            .whileTrue(driveManager.defineDriver(HubAlign()))
        //
        // driverController.b().debounce(0.2).whileTrue(driveManager.defineDriver(TrenchAlign()))
        //
        driverController.x().debounce(0.2).whileTrue(driveManager.defineDriver(LockDrive()))
        //        driverController
        //            .y()
        //            .whileTrue(
        //                driveManager.defineDriver(
        //                    HubDistanceController(
        //                        desiredDistance = { hubDistance.meters },
        //                        moveAround = { translationX },
        //                    )
        //                )
        //            )

        // Note: There is no method to directly get the angle of the POV
        val forward = 0.degrees
        val step = (-45).degrees
        driverController.povUp().whileTrue(driveManager.defineDriver(CardinalAlign { forward }))
        driverController
            .povUpRight()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step }))
        driverController
            .povRight()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 2.0 }))
        driverController
            .povDownRight()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 3.0 }))
        driverController
            .povDown()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 4.0 }))
        driverController
            .povDownLeft()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 5.0 }))
        driverController
            .povLeft()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 6.0 }))
        driverController
            .povUpLeft()
            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 7.0 }))

        trenchDriveTrigger.onTrue(rumble(GenericHID.RumbleType.kBothRumble, 0.5, 0.2.seconds))

        /** Shooter */
        //        operatorController
        //            .axisLessThan(operatorController.throttleChannel, 0.5)
        //            .whileTrue(
        //                Shooter.runAtSpeed { (6000 * ((1.0 - operatorController.throttle * 2) /
        // 3)).RPM }
        //            )
        operatorController
            .axisLessThan(operatorController.throttleChannel, 0.5)
            .whileTrue(
                Shooter.runAtSpeed {
                    (desiredRPM * ((1.0 - operatorController.throttle * 2) / 3)).RPM
                }
            )

        // (0.5 - operatorController.throttle) * (2 / 3)
        //        operatorController
        //            .axisLessThan(operatorController.throttleChannel, 0.5)
        //            .whileTrue(Shooter.runAtPower(1.0))

        var desiredHoodPosition = 0.5.radians
        operatorController
            .button(4)
            .whileTrue(
                RunCommand({
                    desiredHoodPosition =
                        (desiredHoodPosition.asRadians - 0.025)
                            .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
                            .radians
                    println(desiredHoodPosition)
                })
            )

        operatorController
            .button(6)
            .whileTrue(
                RunCommand({
                    desiredHoodPosition =
                        (desiredHoodPosition.asRadians + 0.025)
                            .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
                            .radians
                    println(desiredHoodPosition)
                })
            )

        operatorTrigger
            .and(driverController.a().negate())
            .whileTrue(
                SequentialCommandGroup(
                    Shooter.Hood.moveToPosition { desiredHoodAngle.radians },
                    Shooter.Feeder.getJiggyWithIt()
                        .alongWith(Shooter.Hood.holdPosition { desiredHoodAngle.radians }),
                )
            )
        operatorTrigger
            .and(driverController.a())
            .whileTrue(SequentialCommandGroup(Shooter.Feeder.getJiggyWithIt()))
        operatorController
            .button(2)
            .or(operatorController.button(8))
            .and(operatorTrigger.negate())
            .whileTrue(Shooter.Hood.holdPosition { desiredHoodPosition })
        //        operatorController.button(4).whileTrue(Shooter.Hood.doRunAtkS())
        //        operatorController.button(3).whileTrue(Shooter.Hood.stabilize())
        //        operatorController.button(6).whileTrue(Shooter.Hood.setDownAndReZero())

        /** Intake */
        highHatBack.whileTrue(Intake.runAtPower(1.0))
        highHatForward.whileTrue(Intake.runAtPower(-1.0))

        operatorController.button(5).whileTrue(Intake.Pivot.runAtPower(1.0))
        operatorController.button(3).whileTrue(Intake.Pivot.runAtPower(-1.0))

        operatorController.button(11).whileTrue(Shooter.Hood.runAtDashboardVoltage())
        operatorController.button(12).whileTrue(Shooter.runAtPower { desiredShooterPower })

        //
        // .whileTrue(Intake.Pivot.runAtkS())
        //        operatorController.axisGreaterThan(0,
        // 0.75).whileTrue(Intake.Pivot.runAtPower(-1.0))
        //        operatorController.axisLessThan(0, -0.75).whileTrue(Intake.Pivot.runAtPower(1.0))

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
        SmartDashboard.putData("SysIdCommands/Shooter/HoodAngles", collectShooterAngles())
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
    private val translationX
        get() = process(driverController.leftX, power = 1.5)

    /**
     * Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up)
     */
    private val translationY
        get() = process(driverController.leftY, power = 1.5)

    /**
     * Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1
     * (right)
     */
    private val turn
        get() = process(driverController.rightX, power = 1.5)

    private val rightTrigger
        get() = driverController.rightTriggerAxis

    private val resetGyro: Trigger = driverController.rightBumper()

    private val highHatForward: Trigger = operatorController.pov(0)
    private val highHatBack: Trigger = operatorController.pov(180)
    private val operatorTrigger: Trigger = operatorController.trigger()

    /** Rumbles the driver controller continuously until interrupted. */
    private fun rumble(side: GenericHID.RumbleType, power: Double): Command =
        runEnd(
            { driverController.setRumble(side, power) },
            { driverController.setRumble(side, 0.0) },
        )

    /** Rumbles the driver controller until the given time has elapsed. */
    private fun rumble(side: GenericHID.RumbleType, power: Double, time: Time): Command =
        rumble(side, power).withTimeout(time.asSeconds)
}
