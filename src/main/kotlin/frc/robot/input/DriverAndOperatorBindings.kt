package frc.robot.input

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.vector2
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.swerve.CardinalAlign
import frc.robot.commands.swerve.HubAlign
import frc.robot.commands.swerve.HubDistanceController
import frc.robot.commands.swerve.LockDrive
import frc.robot.commands.swerve.TeleopDrive
import frc.robot.commands.swerve.TrenchAlign
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.HedgieHelmet
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

fun OI.driverAndOperatorBindings() {
    /** Drivetrain */
    // Reset Gyro
    resetGyro
        .debounce(0.5)
        .onTrue(
            InstantCommand({ Drivetrain.zeroGyro() }, Drivetrain)
                .andThen(rumble(GenericHID.RumbleType.kLeftRumble, 0.25, 0.2.seconds))
        )
    // Default drive
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

    // Turn hub align
    driverController
        .a()
        .or(driverController.y())
        .whileTrue(driveManager.defineDriver(HubAlign()))
        .and(HedgieHelmet.trenchDriveTrigger.negate())
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

    // Note: There is no method to directly get the angle of the POV
    val forward = 0.degrees
    // val step = (-45).degrees
    Trigger { driverController.hid.pov != -1 }
        .whileTrue(
            driveManager.defineDriver(CardinalAlign { forward - driverController.hid.pov.degrees })
        )
    //        driverController
    //            .po()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward }))
    //        driverController
    //            .povUpRight()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step }))
    //        driverController
    //            .povRight()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 2.0 }))
    //        driverController
    //            .povDownRight()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 3.0 }))
    //        driverController
    //            .povDown()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 4.0 }))
    //        driverController
    //            .povDownLeft()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 5.0 }))
    //        driverController
    //            .povLeft()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 6.0 }))
    //        driverController
    //            .povUpLeft()
    //            .whileTrue(driveManager.defineDriver(CardinalAlign { forward + step * 7.0 }))

    HedgieHelmet.trenchDriveTrigger.onTrue(
        rumble(GenericHID.RumbleType.kBothRumble, 0.5, 0.2.seconds)
    )

    /** Shooter */
    operatorController
        .axisLessThan(operatorController.throttleChannel, 0.5)
        .whileTrue(
            Shooter.runAtSpeed { (desiredRPM * ((1.0 - operatorController.throttle * 2) / 3)).RPM }
        )

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
}
