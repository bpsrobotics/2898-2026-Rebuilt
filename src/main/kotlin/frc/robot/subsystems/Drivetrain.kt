package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import beaverlib.utils.Units.Linear.feetPerSecond
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Linear.meters
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.vision.TargetPoseProvider
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.io.File
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

object Drivetrain : SubsystemBase() {
    object Constants {
        val MAX_SPEED_MPS = (15.1).feetPerSecond.asMetersPerSecond
        val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = (Math.PI).radiansPerSecond.asRadiansPerSecond
        // Chassis configuration (left to right dist of center of the wheels)
        private val TRACK_WIDTH = Units.inchesToMeters(11.5)

        // Distance between centers of right and left wheels on robot (front to back dist)
        private val WHEEL_BASE = Units.inchesToMeters(11.5)

        // Distance between front and back wheels on robot: CHANGE TO MATCH WITH ROBOT
        val DRIVE_KINEMATICS =
            arrayOf(
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            )
        // YAGSL `File` Configs
        val DRIVE_CONFIG: File = File(Filesystem.getDeployDirectory(), "swerve1")

        val ROBOT_WIDTH = 29.inches
        // val BUMPER_WIDTH = 35.inches
    }

    private val swerveDrive: SwerveDrive

    /** The maximum speed of the swerve drive */
    val maximumSpeed = Constants.MAX_SPEED_MPS
    val maxAngularSpeed = Constants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND

    /** SwerveModuleStates publisher for swerve display */
    private val swerveStatePublisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct)
            .publish()
    private val posePublisher: StructPublisher<Pose2d> =
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish()

    private val targetPosePublisher: StructPublisher<Pose2d> =
        NetworkTableInstance.getDefault().getStructTopic("TargetPose", Pose2d.struct).publish()

    var updateVisionOdometry = true

    private val targetPoseProvider =
        TargetPoseProvider(FieldMapREBUILTWelded.teamHub.center, 2.meters) { 0.radians }

    init {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects
        // being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH

        swerveDrive =
            SwerveParser(Constants.DRIVE_CONFIG).createSwerveDrive(Constants.MAX_SPEED_MPS)

        // Set YAGSL preferences
        swerveDrive.setHeadingCorrection(false)
        // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false)
        // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations
        // since it causes discrepancies not seen in real life.
        swerveDrive.setMotorIdleMode(false)

        swerveDrive.setGyroOffset(Rotation3d(0.0, 0.0, PI))

        // Updates odometry whenever vision sees apriltag
        Vision.listeners.add(
            "UpdateOdometry",
            fun(result, camera) {
                if (!updateVisionOdometry) return
                if (result.targets.isEmpty()) return
                if (
                    !result.multitagResult.isPresent && (result.targets.first().poseAmbiguity > 0.3)
                )
                    return
                val newPose = camera.getMultiTagPoseWithFallback(result) ?: return
                addVisionMeasurement(newPose.toPose2d(), result.timestampSeconds)
            },
        )
        setVisionMeasurementStdDevs(3.0, 4.0, 5.0)
        // setupPathPlanner()

        targetPoseProvider.initialize()
    }

    fun doEnableVisionOdometry(enable: Boolean = true) =
        InstantCommand({ updateVisionOdometry = enable })

    override fun periodic() {
        posePublisher.set(pose)
        swerveStatePublisher.set(swerveDrive.states)
        targetPosePublisher.set(targetPoseProvider.getPose())
        Vision.setAllCameraReferences(Pose3d(pose))
        SmartDashboard.putNumber("Odometry/X", pose.x)
        SmartDashboard.putNumber("Odometry/Y", pose.y)
        SmartDashboard.putNumber("Odometry/HEADING", pose.rotation.radians)
    }

    fun getAlliance(): DriverStation.Alliance =
        DriverStation.getAlliance().getOrNull() ?: DriverStation.Alliance.Blue

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
     * Return SysID command for drive motors from YAGSL
     *
     * @return A command that SysIDs the drive motors.
     */
    fun sysIdDriveMotors(): Command? {
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
     * @return A command that SysIDs the angle motors.
     */
    fun sysIdAngleMotors(): Command {
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
     * @param fieldOriented Whether the robot's motion should be field-oriented or robot-oriented.
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

    /** Method to zero the gyro. */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /** Method to get the current heading (yaw) of the robot. */
    private val heading: Rotation2d
        get() = swerveDrive.yaw

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and Rotational velocity.
     *
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param angle The desired rotational velocity of the robot.
     * @return The generated ChassisSpeeds object.
     */
    @Suppress("unused")
    fun getTargetSpeeds(vForward: Double, vSide: Double, angle: Rotation2d): ChassisSpeeds =
        swerveDrive.swerveController.getTargetSpeeds(
            vForward,
            vSide,
            angle.radians,
            heading.radians,
            maximumSpeed,
        )

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and angle X and Y components.
     *
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param headingX The desired X component of the angle.
     * @param headingY The desired Y component of the angle.
     * @return The generated ChassisSpeeds object.
     */
    @Suppress("unused")
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        headingX: Double,
        headingY: Double,
    ): ChassisSpeeds =
        swerveDrive.swerveController.getTargetSpeeds(
            vForward,
            vSide,
            headingX,
            headingY,
            heading.radians,
            maximumSpeed,
        )

    /** Returns the current field oriented velocity of the robot. */
    val fieldVelocity: ChassisSpeeds
        get() = swerveDrive.fieldVelocity

    /** Returns the current robot oriented velocity of the robot. */
    val robotVelocity: ChassisSpeeds
        get() = swerveDrive.robotVelocity

    /** Method to toggle the lock position of the swerve drive to prevent motion. */
    fun lock() {
        swerveDrive.lockPose()
    }

    /**
     * Add a vision measurement to the swerve drive's pose estimator.
     *
     * @param measurement The pose measurement to add.
     * @param timestamp The timestamp of the pose measurement.
     */
    private fun addVisionMeasurement(measurement: Pose2d, timestamp: Double) {
        swerveDrive.addVisionMeasurement(measurement, timestamp)
    }

    /**
     * Set the standard deviations of the vision measurements.
     *
     * @param stdDevX The standard deviation of the X component of the vision measurements.
     * @param stdDevY The standard deviation of the Y component of the vision measurements.
     * @param stdDevTheta The standard deviation of the rotational component of the vision
     *   measurements.
     */
    @Suppress("SameParameterValue")
    private fun setVisionMeasurementStdDevs(stdDevX: Double, stdDevY: Double, stdDevTheta: Double) {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(stdDevX, stdDevY, stdDevTheta)
        )
    }
}
