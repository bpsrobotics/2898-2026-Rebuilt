package frc.robot.commands.autos

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.swerve.MoveTo
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter

val shootPositions =
    mapOf(
        AllianceStationID.Red1 to Vector2(2.33, 6.14),
        AllianceStationID.Red2 to Vector2(2.0, 4.55),
        AllianceStationID.Red3 to Vector2(2.47, 2.33),
        AllianceStationID.Blue1 to Vector2(14.22, 2.22),
        AllianceStationID.Blue2 to Vector2(14.31, 3.3),
        AllianceStationID.Blue3 to Vector2(14.09, 5.19),
    )

fun simpleMoveAndShoot(): Command {
    var pos = Vector2(0.0, 0.0)

    return MoveTo { pos.toPose2d(pos.angleTo(FieldMapREBUILTWelded.teamHub.center)) }
        .beforeStarting({
            pos =
                shootPositions[DriverStation.getRawAllianceStation()]
                    ?: shootPositions[AllianceStationID.Red2]!!
        })
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
            Shooter.stabilize(),
        )
}
