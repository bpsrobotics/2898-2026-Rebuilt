package frc.robot

import beaverlib.controls.PIDConstants
import beaverlib.controls.PathPlannerPID
import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.AngularAcceleration
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.Units.Angular.radiansPerSecond
import beaverlib.utils.Units.Angular.radiansPerSecondSquared
import beaverlib.utils.Units.Linear.Acceleration
import beaverlib.utils.Units.Linear.VelocityUnit
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Linear.metersPerSecond
import beaverlib.utils.Units.Linear.metersPerSecondSquared
import beaverlib.utils.Units.lb
import beaverlib.utils.geometry.vector2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.commands.swerve.HubAlign
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import kotlin.math.PI

object Autos {
    init {
        // Register commands

        // Spins the shooter up to speed
        NamedCommands.registerCommand(
            "SpinUpShooter",
            Shooter.runAtSpeed()
        )

        // Wait until shooter is up to speed, does not spin it up
        NamedCommands.registerCommand(
            "WaitForShooter",
            Shooter.waitSpeed()
        )

        // Lowers the intake
        NamedCommands.registerCommand(
            "ExtendIntake",
            Intake.Pivot.extend()
        )

        // Raises the intake
        NamedCommands.registerCommand(
            "StowIntake",
            Intake.Pivot.stow()
        )

        // Spins the intake
        NamedCommands.registerCommand(
            "RunIntake",
            Intake.runAtPower(1.0)
        )

        // Stops the intake
        NamedCommands.registerCommand(
            "StopIntake",
            Intake.stop()
        )

        // Aligns to hub rotation, positions hood, waits for flywheel, then feeds + jiggles
        NamedCommands.registerCommand(
            "AlignAndShoot",
            buildAlignAndShoot()
        )
    }

    object Constants {
        val robotConfig =
            RobotConfig(
                (120.0).lb.asKilograms,
                Drivetrain.Constants.MAX_SPEED_MPS,
                ModuleConfig(
                    (2.0).inches.asMeters,
                    Drivetrain.Constants.MAX_SPEED_MPS,
                    1.54,
                    DCMotor.getNEO(1).withReduction(6.75),
                    30.0,
                    1,
                ),
                *Drivetrain.Constants.DRIVE_KINEMATICS,
            )

        // const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0
        // private const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI
        // private const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI

        val translationPIDConstants = PIDConstants(5.0, 0.0, 0.0)
        val rotationPIDConstants = PIDConstants(0.01, 0.0, 0.0)

        // AlignAndShoot timeouts
        const val ALIGN_TIMEOUT_SECONDS = 1.0
        const val SHOOT_TIMEOUT_SECONDS = 20.0
        const val WAIT_FOR_SPEED_TIMEOUT_SECONDS = 2.0

        // Constraint for the motion profiled robot angle controller
        // val THETA_CONTROLLER_CONSTRAINTS =
        //     TrapezoidProfile.Constraints(
        //         MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        //         MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED
        //     )
    }

    private var autoCommandChooser: SendableChooser<Command> = SendableChooser()
    val autonomousCommand: Command
        get() = autoCommandChooser.selected

    private val autos: Map<String, Command>

    fun addAutos() {
        autoCommandChooser.setDefaultOption("No Auto", InstantCommand())
        autos.forEach { (name, command) -> autoCommandChooser.addOption(name, command) }
        SmartDashboard.putData("Auto Chooser", autoCommandChooser)
    }

    init {
        AutoBuilder.configure(
            { Drivetrain.pose }, // Robot pose supplier
            Drivetrain::
                resetOdometry, // Method to reset odometry (will be called if your auto has a
            // starting pose)
            { Drivetrain.robotVelocity }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            Drivetrain.driveConsumer, // Method that will drive the robot given ROBOT RELATIVE
            // ChassisSpeeds.
            // Also, optionally outputs individual module feedforwards
            PPHolonomicDriveController( // PPHolonomicController is the built-in path following
                // controller for holonomic drive trains
                Constants.translationPIDConstants.PathPlannerPID, // Translation PID constants
                Constants.rotationPIDConstants.PathPlannerPID,
            ),
            Constants.robotConfig,
            { Drivetrain.getAlliance() == DriverStation.Alliance.Red },
            Drivetrain, // Reference to this subsystem to set requirements
        )

        autos = mapOf<String, Command>(
            "Spin Up Flywheel" to AutoBuilder.buildAuto("SpinUpFlywheel"),
            "Drive Forward" to AutoBuilder.buildAuto("DriveForward"),
            "Center - Drive Back and Shoot" to AutoBuilder.buildAuto("Center-DriveBackShoot"),
            "Left Trench - Collect Fuel Safe" to AutoBuilder.buildAuto("LeftTrench-CollectFuelSafe"),
            "Left Trench - Collect Fuel Double Pass" to AutoBuilder.buildAuto("LeftTrench-CollectFuelDoublePass"),
        )
    }

    /**
     * Builds the AlignAndShoot command
     * 1. Rotate to face hub + move hood to position
     * 2. Wait until the flywheel is at speed
     * 3. Run feeder to shoot
     */
    private fun buildAlignAndShoot(): Command {
        val rotationPID = HubAlign.createRotationPID()

        // Part 1
        val alignAndPositionHood = Drivetrain.run {
            rotationPID.setpoint = HubAlign.hubSetpointRadians
            val omega = rotationPID.calculate(Drivetrain.pose.rotation.radians)
            Drivetrain.driveFieldOriented(ChassisSpeeds(0.0, 0.0, omega))
        }.beforeStarting({ rotationPID.reset() })
            .withTimeout(Constants.ALIGN_TIMEOUT_SECONDS)
            .alongWith(
                Shooter.Hood.holdPosition {
                    Shooter.Hood.Constants.kinematics
                        .calculate(
                            Drivetrain.pose.vector2.distance(FieldMapREBUILTWelded.teamHub.center)
                        )
                        .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
                        .radians
                }.withTimeout(Constants.ALIGN_TIMEOUT_SECONDS)
            )

        // Part 2
        val waitForSpeed = Shooter.waitSpeed()
            .withTimeout(Constants.WAIT_FOR_SPEED_TIMEOUT_SECONDS)

        // Part 3
        val shootPhase = Shooter.Feeder.getJiggyWithIt(1.0)
            .alongWith(
                Shooter.Hood.holdPosition {
                    Shooter.Hood.Constants.kinematics
                        .calculate(
                            Drivetrain.pose.vector2.distance(FieldMapREBUILTWelded.teamHub.center)
                        )
                        .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
                        .radians
                }
            )
            .withTimeout(Constants.SHOOT_TIMEOUT_SECONDS)

        return alignAndPositionHood
            .andThen(waitForSpeed)
            .andThen(shootPhase)
    }

    @Suppress("unused")
    fun generatePath(
        vararg pose2dWaypoints: Pose2d,
        maxVelocity: VelocityUnit = 3.0.metersPerSecond,
        maxAcceleration: Acceleration = 3.0.metersPerSecondSquared,
        maxAngularVelocity: AngularVelocity = (2 * PI).radiansPerSecond,
        maxAngularAcceleration: AngularAcceleration = (4 * PI).radiansPerSecondSquared,
    ): PathPlannerPath {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use
        // holonomic rotation.
        val waypoints = PathPlannerPath.waypointsFromPoses(pose2dWaypoints.asList())

        val constraints =
            PathConstraints(
                maxVelocity.asMetersPerSecond,
                maxAcceleration.asMetersPerSecondSquared,
                maxAngularVelocity.asRadiansPerSecond,
                maxAngularAcceleration.asRadiansPerSecondSquared,
            ) // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also
        // use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        val path =
            PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so
                // can be null for on-the-fly paths.
                GoalEndState(
                    0.0,
                    Rotation2d.fromDegrees(-90.0),
                ), // Goal end state. You can set a holonomic rotation here. If using a differential
                // drivetrain, the rotation will have no effect.
            )
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true
        return path
    }

    @Suppress("unused")
    fun pathFindToPose(
        pose: Pose2d,
        maxVelocity: VelocityUnit = 1.0.metersPerSecond,
        maxAcceleration: Acceleration = 1.0.metersPerSecondSquared,
        maxAngularVelocity: AngularVelocity = (2 * PI).radiansPerSecond,
        maxAngularAcceleration: AngularAcceleration = (4 * PI).radiansPerSecondSquared,
    ): Command =
        AutoBuilder.pathfindToPose(
            pose,
            PathConstraints(
                maxVelocity.asMetersPerSecond,
                maxAcceleration.asMetersPerSecondSquared,
                maxAngularVelocity.asRadiansPerSecond,
                maxAngularAcceleration.asRadiansPerSecondSquared,
            ),
        ) // The constraints for this path.
}
