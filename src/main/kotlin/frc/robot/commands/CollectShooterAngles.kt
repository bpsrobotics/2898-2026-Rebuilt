package frc.robot.commands

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.radians
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.commands.swerve.MoveTo
import frc.robot.subsystems.Shooter

infix fun ClosedRange<Double>.step(step: Double): Iterable<Double> {
    require(start.isFinite())
    require(endInclusive.isFinite())
    require(step > 0.0) { "Step must be positive, was: $step." }

    // Generate a sequence of values
    val sequence =
        generateSequence(start) { previous ->
            if (previous == Double.POSITIVE_INFINITY) return@generateSequence null
            val next = previous + step
            if (next > endInclusive) null else next
        }
    return sequence.asIterable()
}

fun collectShooterAngles(): Command =
    sequence(
        *(2.0..3.0 step 0.25)
            .map { distance ->
                var currentMinAngle = Shooter.Hood.Constants.DOWN_POSITION
                var currentMaxAngle = Shooter.Hood.Constants.TOP_POSITION
                var angle = (currentMinAngle + currentMaxAngle) / 2.0
                InstantCommand({
                        currentMinAngle = Shooter.Hood.Constants.DOWN_POSITION
                        currentMaxAngle = Shooter.Hood.Constants.TOP_POSITION
                    })
                    .andThen(
                        MoveTo {
                                (FieldMapREBUILTWelded.teamHub.center + Vector2(-distance, 0.0))
                                    .toPose2d(0.radians)
                            }
                            .alongWith(
                                Shooter.waitSpeed(),
                                InstantCommand({
                                    angle = (currentMinAngle + currentMaxAngle) / 2.0
                                    SmartDashboard.putNumber(
                                        "ShooterHoodTuning/Angle",
                                        angle.asRadians,
                                    )
                                    SmartDashboard.putBoolean(
                                        "ShooterHoodTuning/notFarEnough",
                                        false,
                                    )
                                    SmartDashboard.putBoolean("ShooterHoodTuning/tooFar", false)
                                }),
                            )
                            .andThen(
                                Shooter.Hood.moveToPosition { angle },
                                Shooter.Hood.stabilize()
                                    .alongWith(Shooter.Feeder.getJiggyWithIt())
                                    .withDeadline(
                                        waitUntil {
                                            SmartDashboard.getBoolean(
                                                "ShooterHoodTuning/notFarEnough",
                                                false,
                                            ) ||
                                                SmartDashboard.getBoolean(
                                                    "ShooterHoodTuning/tooFar",
                                                    false,
                                                )
                                        }
                                    ),
                                InstantCommand({
                                    if (
                                        SmartDashboard.getBoolean("ShooterHoodTuning/tooFar", false)
                                    ) {
                                        currentMinAngle = angle
                                    } else if (
                                        SmartDashboard.getBoolean(
                                            "ShooterHoodTuning/notFarEnough",
                                            false,
                                        )
                                    ) {
                                        currentMaxAngle = angle
                                    }
                                }),
                            )
                    )
            }
            .toTypedArray()
    )
