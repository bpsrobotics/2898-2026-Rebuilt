package frc.robot.engine.QuillT

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand

object QuillT {
    fun parseNetworkPath(path: String, constraints: PathConstraints): Command {
        val pathArray: Array<Double> =
            SmartDashboard.getNumberArray("QuillT/Auto/execution/${path}", arrayOf())
        if (pathArray.isEmpty() || pathArray.size % 7 != 0) return InstantCommand()
        var waypoints = mutableListOf<Waypoint>()
        val goalEndState = GoalEndState(0.0, Rotation2d())
        for (i in 0..<(pathArray.size / 7)) {
            val prevControl: Translation2d? =
                if (pathArray[i] < -500 && pathArray[i + 1] < -500.0) {
                    null
                } else {
                    Translation2d(pathArray[i], pathArray[i + 1])
                }
            val nextControl: Translation2d? =
                if (pathArray[i + 2] < -500 && pathArray[i + 3] < -500.0) {
                    null
                } else {
                    Translation2d(pathArray[i + 2], pathArray[i + 3])
                }
            if (i == (pathArray.size / 7) - 1) {
                GoalEndState(0.0, Rotation2d.fromRadians(pathArray[i + 6]))
            }

            waypoints.add(
                Waypoint(
                    prevControl,
                    Translation2d(pathArray[i + 4], pathArray[i + 5]),
                    nextControl,
                )
            )
        }
        return AutoBuilder.followPath(PathPlannerPath(waypoints, constraints, null, goalEndState))
    }

    fun parseNetwork(constraints: PathConstraints): Command {
        val execution = SmartDashboard.getStringArray("QuillT/Auto/execution", arrayOf())
        val returnCommand = InstantCommand()
        if (execution.isEmpty()) return InstantCommand()
        for (i in 0..execution.size) {
            if (i % 2 != 0) continue
            if (execution[i] == "Path") {
                returnCommand.andThen(parseNetworkPath(execution[i + 1], constraints))
            }
        }
        return returnCommand
    }
}
