package frc.robot.commands.autos

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.swerve.MoveTo
import frc.robot.engine.DashboardNumber
import frc.robot.input.OI
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter

val shootVectors =
    mapOf(
        1 to Vector2(-0.7071067811865476, 0.7071067811865476),
        2 to Vector2(-0.9615239476408232, 0.27472112789737807),
        3 to Vector2(-0.9363291775690445, -0.3511234415883917),
    )

val distance by DashboardNumber(2.0, "simpleMoveAndShoot")
val desiredPosePublisher: StructPublisher<Pose2d?> = NetworkTableInstance
    .getDefault()
    .getStructTopic("simpleMoveAndShoot/desiredPose", Pose2d.struct)
    .publish()
val overrideTargetPosition by DashboardNumber(0.0, "simpleMoveAndShoot")

fun simpleMoveAndShoot(): Command {
    var pos = Vector2(0.0, 0.0)

    return MoveTo {
        val pose = pos.toPose2d((pos.angleTo(FieldMapREBUILTWelded.teamHub.center) + 180.degrees).getCoterminal())
        desiredPosePublisher.set(pose)
        pose
    }
        .solitarily()
        .beforeStarting({
            pos =
                FieldMapREBUILTWelded.teamHub.center +
                    (shootVectors[if (overrideTargetPosition > 0) overrideTargetPosition else DriverStation.getLocation().orElse(2)] ?: shootVectors[2]!!) *
                        distance *
                        -OI.reverseDrive
        })
        .withTimeout(5.0)
        .andThen(Shooter.Feeder.getJiggyWithIt(1.0))
        .alongWith(
            Shooter.Hood.holdPosition {
                Shooter.Hood.Constants.kinematics
                    .calculate(
                        Drivetrain.pose.vector2.distance(FieldMapREBUILTWelded.teamHub.center)
                    )
                    .clamp(0.0, Shooter.Hood.Constants.TOP_POSITION.asRadians)
                    .radians
            },
            Shooter.runAtSpeed(),
        )
}
