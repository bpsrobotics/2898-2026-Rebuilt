package frc.robot.commands

import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Time
import beaverlib.utils.Units.seconds
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.Gate
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Tunnel

fun DoOpenloopIntake(
    intakePower: Double = 0.45,
    tunnelPower: Double = 0.4,
    gateReverseSpeed: Double = 0.05,
    time: Time? = null,
) =
    ParallelRaceGroup(
        Intake.runAtPowerCommand(intakePower, time),
        Tunnel.runAtPowerCommand(tunnelPower),
        Gate.runAtPowerCommand(-gateReverseSpeed),
    )

fun DoOpenLoopShooter(
    shooterPower: Double = 0.6,
    gatePower: Double = 0.4,
    tunnelPower: Double = 0.4,
    spinupTime: Time = 0.4.seconds,
    shootTime: Time? = null,
) =
    SequentialCommandGroup(
        Shooter.openloopSpinup(shooterPower, spinupTime),
        ParallelRaceGroup(
            Shooter.openloopShoot(shooterPower, shootTime),
            Gate.runAtPowerCommand(gatePower),
            Tunnel.runAtPowerCommand(tunnelPower),
        ),
    )

fun DoShoot(
    shooterSpeed: () -> AngularVelocity,
    tunnelPower: Double = 0.4,
    gatePower: Double = 0.4,
    shootTime: Time? = null,
) =
    SequentialCommandGroup(
        Shooter.spinup(shooterSpeed),
        ParallelRaceGroup(
            Shooter.shoot(shooterSpeed, time = shootTime),
            Tunnel.runAtPowerCommand(tunnelPower),
            SequentialCommandGroup(Gate.runAtPowerCommand(gatePower, 0.5.seconds), WaitCommand(0.5)).repeatedly(),
        ),
    )

fun DoShootIntake(
    shooterSpeed: () -> AngularVelocity,
    intakePower: Double = 0.45,
    tunnelPower: Double = 0.4,
    gatePower: Double = 0.4,
    shootTime: Time? = null,
) =
    SequentialCommandGroup(
        ParallelRaceGroup(
            Shooter.spinup(shooterSpeed),
            Intake.runAtPowerCommand(intakePower),
            Tunnel.runAtPowerCommand(tunnelPower),
        ),
        ParallelRaceGroup(
            Shooter.shoot(shooterSpeed, time = shootTime),
            Intake.runAtPowerCommand(intakePower),
            Tunnel.runAtPowerCommand(tunnelPower),
            SequentialCommandGroup(Gate.runAtPowerCommand(gatePower, 0.5.seconds), WaitCommand(0.5)).repeatedly(),
        ),
    )

fun DoRunAllRobot(
    intakePower: Double,
    tunnelPower: Double,
    gatePower: Double,
    shooterPower: Double,
    spinupTime: Time,
    shootTime: Time? = null,
) =
    SequentialCommandGroup(
        ParallelRaceGroup(
            Shooter.openloopSpinup(shooterPower, spinupTime),
            Intake.runAtPowerCommand(intakePower),
            Tunnel.runAtPowerCommand(tunnelPower),
        ),
        ParallelRaceGroup(
            Shooter.openloopShoot(shooterPower, shootTime),
            Intake.runAtPowerCommand(intakePower),
            Tunnel.runAtPowerCommand(tunnelPower),
            Gate.runAtPowerCommand(gatePower),
        ),
    )

fun DoOutakeFullRobot(
    intakePower: Double = 0.45,
    tunnelPower: Double = 0.2,
    gatePower: Double = 0.2,
    shooterPower: Double = 0.1,
    time: Time? = null,
) =
    ParallelRaceGroup(
        Shooter.openloopShoot(-shooterPower, time),
        Intake.runAtPowerCommand(-intakePower),
        Tunnel.runAtPowerCommand(-tunnelPower),
        Gate.runAtPowerCommand(-gatePower),
    )
