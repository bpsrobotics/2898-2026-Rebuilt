// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import frc.robot.OI.configureBindings
import frc.robot.OI.rightTrigger
import frc.robot.OI.translationX
import frc.robot.OI.translationY
import frc.robot.OI.turnX
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private val m_exampleSubsystem = ExampleSubsystem()
    // Replace with CommandPS4Controller or CommandJoystick if needed

    val alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)

    val reverseDrive =
        if (alliance == DriverStation.Alliance.Red) {
            1.0
        } else {
            -1.0
        }

    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { translationY * reverseDrive },
            { translationX * reverseDrive },
            { -turnX },
            { true },
            { rightTrigger },
        )

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        initializeObjects()

        Drivetrain.defaultCommand = teleopDrive

        Autos.addAutos()

        configureBindings()
    }

    private fun initializeObjects() {
        Drivetrain
        Vision
    }
}
