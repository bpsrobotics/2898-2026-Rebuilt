package frc.robot.commands.autos

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.seconds
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Autos
import frc.robot.commands.autos.AutoShootCarrotsStuff.desiredShooterSpeed
import frc.robot.commands.autos.AutoShootCarrotsStuff.targetPose
import frc.robot.commands.vision.AlignOdometry
import frc.robot.engine.FieldMap
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Gate
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Tunnel
import kotlin.math.sqrt

val AutoAlign =
    SequentialCommandGroup(
        Drivetrain.doEnableVisionOdometry(false),
        DeferredCommand({ AlignOdometry(targetPose) }, setOf(Drivetrain)),
        Drivetrain.doEnableVisionOdometry(true),
    )

val AutoShootCarrots =
    DeferredCommand(
        {
            AlignOdometry(targetPose)
                .alongWith(Shooter.spinup(desiredShooterSpeed))
                .andThen(
                    ParallelRaceGroup(
                        Shooter.shoot(desiredShooterSpeed, time = 3.0.seconds),
                        ParallelRaceGroup(
                                Gate.runAtPowerCommand(0.4, 0.2.seconds),
                                Tunnel.runAtPowerCommand(0.4),
                                Intake.runAtPowerCommand(0.4),
                            )
                            .andThen(WaitCommand(0.5))
                            .repeatedly(),
                    )
                )
        },
        setOf(Drivetrain, Shooter, Tunnel, Gate),
    )
val AutoShootThenMovement =
    DeferredCommand(
        {
            AlignOdometry(targetPose)
                .alongWith(Shooter.spinup(desiredShooterSpeed))
                .andThen(
                    ParallelRaceGroup(
                        Shooter.shoot(desiredShooterSpeed, time = 3.0.seconds),
                        ParallelRaceGroup(
                                Gate.runAtPowerCommand(0.4, 0.2.seconds),
                                Tunnel.runAtPowerCommand(0.4),
                                Intake.runAtPowerCommand(0.4),
                            )
                            .andThen(WaitCommand(0.5))
                            .repeatedly(),
                    )
                )
                .andThen(
                    AlignOdometry(
                        Pose2d(
                            Drivetrain.pose.x,
                            FieldMap.teamFeederStation.center.y +
                                Drivetrain.Constants.BumperWidth.asMeters,
                            Rotation2d(),
                        )
                    )
                )
                .andThen(
                    AlignOdometry(Pose2d(FieldMap.FieldCenter.x, Drivetrain.pose.y, Rotation2d()))
                )
        },
        setOf(Drivetrain, Shooter, Tunnel, Gate),
    )
val PathPlanAutoShootCarrots =
    DeferredCommand(
        {
            Autos.pathFindToPose(targetPose)
                .alongWith(Shooter.spinup(desiredShooterSpeed))
                .andThen(
                    ParallelRaceGroup(
                        Shooter.shoot(desiredShooterSpeed, time = 2.0.seconds),
                        Gate.runAtPowerCommand(0.4),
                    )
                )
        },
        setOf(Drivetrain, Shooter, Tunnel, Gate),
    )

object AutoShootCarrotsStuff {

    val targetPose: Pose2d
        get() =
            (((Drivetrain.pose.vector2 - FieldMap.teamFeederStation.center).unit *
                    ((FieldMap.ZooWidth / 2) * sqrt(2.0 + 0.1) +
                            Drivetrain.Constants.BumperWidth / 2)
                        .asMeters) +
                    FieldMap.teamFeederStation
                        .center) // Get the closest pose that is proper distance from the
                // feeder
                .toPose2d(Drivetrain.pose.vector2.angleTo(FieldMap.teamFeederStation.center))

    val desiredShooterSpeed = { 3500.RPM }
}

var command: Command = InstantCommand()
