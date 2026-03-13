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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
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

@Suppress("Unused")
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

    Trigger { driverController.hid.pov != -1 }
        .whileTrue(driveManager.defineDriver(CardinalAlign { -driverController.hid.pov.degrees }))

    HedgieHelmet.trenchDriveTrigger.onTrue(
        rumble(GenericHID.RumbleType.kBothRumble, 0.5, 0.2.seconds)
    )

    /** Shooter */
    operatorController
        .axisLessThan(operatorController.throttleChannel, 0.5)
        .whileTrue(
            Shooter.runAtSpeed { (desiredRPM * ((1.0 - operatorController.throttle * 2) / 3)).RPM }
        )

    desiredHoodAngle = 1.3
    operatorController
        .button(4)
        .whileTrue(
            RunCommand({
                desiredHoodAngle =
                    (desiredHoodAngle - 0.025)
                        .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
            })
        )

    operatorController
        .button(6)
        .whileTrue(
            RunCommand({
                desiredHoodAngle =
                    (desiredHoodAngle + 0.025)
                        .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
            })
        )

    operatorTrigger
        .and(driverController.a().negate())
        .whileTrue(
            SequentialCommandGroup(
                (Shooter.Hood.moveToPosition { desiredHoodAngle.radians }.deadlineFor(Intake.Pivot.runToPosition(Intake.Pivot.Constants.FEEDER_POSITION))),
                Shooter.Feeder.runAtPower(1.0).alongWith(Shooter.Hood.holdPosition { desiredHoodAngle.radians }).alongWith(Intake.Pivot.getJiggyWithIt()),
            )
        )
    operatorTrigger
        .and(driverController.a())
        .whileTrue(
            WaitUntilCommand { Shooter.Hood.atSetpoint }
                .deadlineFor(Intake.Pivot.runToPosition(Intake.Pivot.Constants.FEEDER_POSITION))
                .andThen(
                    Shooter.Feeder.getJiggyWithIt(1.0).alongWith(Intake.Pivot.getJiggyWithIt())
                )
        )

    operatorController.button(7).whileTrue(Shooter.Hood.stabilize())

    operatorController
        .button(2)
        .or(operatorController.button(8))
        .and(operatorTrigger.negate())
        .whileTrue(Shooter.Hood.holdPosition { desiredHoodAngle.radians })
    //        operatorController.button(4).whileTrue(Shooter.Hood.doRunAtkS())
    //        operatorController.button(3).whileTrue(Shooter.Hood.stabilize())
    //        operatorController.button(6).whileTrue(Shooter.Hood.setDownAndReZero())

    /** Intake */
    highHatBack.whileTrue(Intake.runAtPower(0.75))
    highHatForward.whileTrue(Intake.runAtPower(-0.75).alongWith(Shooter.Feeder.runAtPower(-0.1)))

    operatorController
        .axisLessThan(1, -0.5)
        .and(operatorTrigger.negate())
        .onTrue(Intake.Pivot.setSetpoint(Intake.Pivot.Constants.EXTENDED_POSITION))
    operatorController
        .axisGreaterThan(1, 0.5)
        .and(operatorTrigger.negate())
        .onTrue(Intake.Pivot.setSetpoint(Intake.Pivot.Constants.STOWED_POSITION))
    operatorController.button(11).whileTrue(Shooter.Feeder.getJiggyWithIt(1.0))

    // operatorController.button(12).whileTrue(Shooter.runAtPower { desiredShooterPower })
}
