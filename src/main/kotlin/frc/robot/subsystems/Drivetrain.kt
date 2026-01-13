// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import beaverlib.utils.Units.Angular.radiansPerSecond
import beaverlib.utils.Units.Linear.feetPerSecond
import beaverlib.utils.Units.Linear.inches
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.io.File

object Drivetrain : SubsystemBase() {
    object Constants {
        val MaxSpeedMetersPerSecond = (15.1).feetPerSecond.asMetersPerSecond
        val MaxAngularSpeedRadiansPerSecond = (Math.PI).radiansPerSecond.asRadiansPerSecond
        // Chassis configuration (left to right dist of center of the wheels)
        val TrackWidth = Units.inchesToMeters(11.5)

        // Distance between centers of right and left wheels on robot (front to back dist)
        val WheelBase = Units.inchesToMeters(11.5)

        // Distance between front and back wheels on robot: CHANGE TO MATCH WITH ROBOT
        val DriveKinematics =
            arrayOf(
                Translation2d(WheelBase / 2, TrackWidth / 2),
                Translation2d(WheelBase / 2, -TrackWidth / 2),
                Translation2d(-WheelBase / 2, TrackWidth / 2),
                Translation2d(-WheelBase / 2, -TrackWidth / 2),
            )
        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve1")

        val RobotWidth = 29.inches
        val BumperWidth = 35.inches
    }

    var swerveDrive: SwerveDrive

    /** The maximum speed of the swerve drive */
    var maximumSpeed = Constants.MaxSpeedMetersPerSecond
    var maxAngularSpeed = Constants.MaxAngularSpeedRadiansPerSecond

    /** SwerveModuleStates publisher for swerve display */
    var swerveStatePublisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct)
            .publish()
    var posePublisher: StructPublisher<Pose2d> =
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

    var updateVisionOdometry = true


    init {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects
        // being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH

        try {
            swerveDrive =
                SwerveParser(Constants.DRIVE_CONFIG)
                    .createSwerveDrive(Constants.MaxSpeedMetersPerSecond)
        } catch (e: Exception) {
            e.printStackTrace()
            throw RuntimeException("error creating swerve", e)
        }
        // Set YAGSL preferences
        swerveDrive.setHeadingCorrection(
            false
        ) // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(
            false
        ) // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations
        // since it causes discrepancies not seen in real life.
        swerveDrive.setMotorIdleMode(false)

        // Updates odometry whenever vision sees apriltag
        Vision.listeners.add(
            "UpdateOdometry",
            fun(result, camera) {
                if(!updateVisionOdometry) return
                if (result.targets.isEmpty()) return
                if (
                    !result.multitagResult.isPresent && (result.targets.first().poseAmbiguity > 0.3)
                )
                    return
                val newPose = camera.getEstimatedRobotPose(result) ?: return
                addVisionMeasurement(
                    newPose.toPose2d(),
                    result.timestampSeconds,
                    !DriverStation.isTeleop(),
                )
            },
        )
        setVisionMeasurementStdDevs(3.0, 4.0, 5.0)

        // setupPathPlanner()

    }

    override fun periodic() {
        posePublisher.set(pose)
        swerveStatePublisher.set(swerveDrive.states)
        Vision.setAllCameraReferences(pose)
        SmartDashboard.putNumber("Odometry/X", pose.x)
        SmartDashboard.putNumber("Odometry/Y", pose.y)
        SmartDashboard.putNumber("Odometry/HEADING", pose.rotation.radians)
    }

    val getAlliance: () -> Boolean = {
        val alliance = DriverStation.getAlliance()
        if (alliance.isPresent) alliance.get() == DriverStation.Alliance.Red else false
    }

    fun driveFieldOriented(speeds: ChassisSpeeds) {
        swerveDrive.driveFieldOriented(speeds)
    }

    fun driveRobotOriented(speeds: ChassisSpeeds) {
        swerveDrive.drive(speeds)
    }

    fun stop() {
        driveRobotOriented(ChassisSpeeds())
    }

    /**
     * Directly send voltage to the drive motors.
     *
     * @param volts The voltage to send to the motors.
     */
    fun setRawMotorVoltage(volts: Double) {
        swerveDrive.modules.forEach { it.driveMotor.voltage = volts }
    }

    /**
     * Return SysID command for drive motors from YAGSL
     *
     * @return A command that SysIDs the drive motors.
     */
    fun sysIdDriveMotor(): Command? {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                SysIdRoutine.Config(),
                this,
                swerveDrive,
                12.0,
                true,
            ),
            3.0,
            5.0,
            3.0,
        )
    }

    /**
     * Return SysID command for angle motors from YAGSL
     *
     * @return A command that SysIDs the angle1 motors.
     */
    fun sysIdAngleMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(SysIdRoutine.Config(), this, swerveDrive),
            3.0,
            5.0,
            3.0,
        )
    }

    /**
     * Advanced drive method that translates and rotates the robot, with a custom center of
     * rotation.
     *
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field oriented or robot oriented.
     * @param centerOfRotation The center of rotation of the robot.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double = 0.0,
        fieldOriented: Boolean = false,
        centerOfRotation: Translation2d = Translation2d(),
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    /**
     * Simple drive method that uses ChassisSpeeds to control the robot.
     *
     * @param velocity The desired ChassisSpeeds of the robot
     */
    fun drive(velocity: ChassisSpeeds, fieldOriented: Boolean = false) {
        if (fieldOriented) swerveDrive.driveFieldOriented(velocity) else swerveDrive.drive(velocity)
    }

    /** A ChassisSpeeds consumer used to drive the robot (Mainly for the purposes of PathPlanner) */
    val driveConsumer: (ChassisSpeeds) -> Unit = { fieldSpeeds: ChassisSpeeds ->
        drive(fieldSpeeds)
    }

    /** Returns the Kinematics object of the swerve drive. */
    val kinematics: SwerveDriveKinematics
        get() = swerveDrive.kinematics

    /**
     * Method to reset the odometry of the robot to a desired pose.
     *
     * @param initialHolonomicPose The desired pose to reset the odometry to.
     */
    fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    /** Returns the current pose of the robot. */
    val pose
        get() = Pose2d(swerveDrive.pose.x, swerveDrive.pose.y, swerveDrive.pose.rotation)

    /** Method to display a desired trajectory to a field2d object. */
    fun postTrajectory(trajectory: Trajectory) {
        swerveDrive.postTrajectory(trajectory)
    }

    /** Method to zero the gyro. */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /**
     * Method to toggle the motor's brakes.
     *
     * @param brake Whether to set the motor's brakes to true or false.
     */
    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    /** Method to get the current heading (yaw) of the robot. */
    val heading: Rotation2d
        get() = swerveDrive.yaw

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and Rotational velocity.
     *
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param angle The desired rotational velocity of the robot.
     * @return The generated ChassisSpeeds object.
     */
    fun getTargetSpeeds(vForward: Double, vSide: Double, angle: Rotation2d): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(
            vForward,
            vSide,
            angle.radians,
            heading.radians,
            maximumSpeed,
        )
    }

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and angle X and Y components.
     *
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param headingX The desired X component of the angle.
     * @param headingY The desired Y component of the angle.
     * @return The generated ChassisSpeeds object.
     */
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        headingX: Double,
        headingY: Double,
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(
            vForward,
            vSide,
            headingX,
            headingY,
            heading.radians,
            maximumSpeed,
        )
    }

    /** Returns the current field oriented velocity of the robot. */
    val fieldVelocity: ChassisSpeeds
        get() = swerveDrive.fieldVelocity

    /** Returns the current robot oriented velocity of the robot. */
    val robotVelocity: ChassisSpeeds
        get() = swerveDrive.robotVelocity

    /** Returns the SwerveController object of the swerve drive. */
    val swerveController: SwerveController
        get() = swerveDrive.swerveController

    /** Returns the SwerveDriveConfiguration object of the swerve drive. */
    val swerveDriveConfiguration: SwerveDriveConfiguration
        get() = swerveDrive.swerveDriveConfiguration

    /** Method to toggle the lock position of the swerve drive to prevent motion. */
    fun lock() {
        swerveDrive.lockPose()
    }

    /** Returns the current pitch of the robot. */
    val pitch: Rotation2d
        get() = swerveDrive.pitch

    /**
     * Add a vision measurement to the swerve drive's pose estimator.
     *
     * @param measurement The pose measurement to add.
     * @param timestamp The timestamp of the pose measurement.
     */
    fun addVisionMeasurement(
        measurement: Pose2d,
        timestamp: Double,
        updateRotation: Boolean = false,
    ) {
        if (updateRotation) swerveDrive.addVisionMeasurement(measurement, timestamp)
        else
            swerveDrive.addVisionMeasurement(
                Pose2d(measurement.x, measurement.y, pose.rotation),
                timestamp,
            )
    }

    /**
     * Set the standard deviations of the vision measurements.
     *
     * @param stdDevX The standard deviation of the X component of the vision measurements.
     * @param stdDevY The standard deviation of the Y component of the vision measurements.
     * @param stdDevTheta The standard deviation of the rotational component of the vision
     *   measurements.
     */
    fun setVisionMeasurementStdDevs(stdDevX: Double, stdDevY: Double, stdDevTheta: Double) {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(stdDevX, stdDevY, stdDevTheta)
        )
    }
}
