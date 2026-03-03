// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.SignalLogger
import frc.robot.OI.DriverOnly
import frc.robot.OI.OI
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.HedgieHelmet
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Vision
import frc.robot.subsystems.VisionTurningHandler

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        SignalLogger.stop()

        initializeObjects()

        Autos.addAutos()

        OI.DriverOnly()
    }

    /**
     * By using the names of the subsystems the JVM will initialize these objects, which can
     * sometimes take a while so it is good to do it before match starts
     */
    @Suppress("UnusedExpression")
    private fun initializeObjects() {
        Drivetrain
        Intake
        Intake.Pivot
        Shooter
        Shooter.Hood
        Shooter.Feeder
        HedgieHelmet
        VisionTurningHandler
        Vision
    }
}
