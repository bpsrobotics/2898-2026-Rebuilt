package frc.robot.subsystems

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.geometry.Raycast2D
import beaverlib.utils.geometry.Vector2
import beaverlib.utils.geometry.vector2
import com.pathplanner.lib.events.TriggerEvent
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.OI.Rumble

object HedgieHelmet {
    val trenchDriveTrigger = Trigger({ willCollideWithTrench()})
    init {
        trenchDriveTrigger.onTrue(Rumble(GenericHID.RumbleType.kBothRumble, 0.5, 0.5))
    }

    fun willCollideWithTrench() : Boolean{
        val robotVelocityVector: Vector2 = Drivetrain.robotVelocity.vector2
        val robotPoseVector: Vector2 = Drivetrain.pose.vector2
        val rayCast: Raycast2D = Raycast2D(robotPoseVector, robotVelocityVector.angle)
        for (trench in FieldMapREBUILTWelded.trenches) {
            val intersection = trench.line.intersection(rayCast) ?: continue
            if(intersection.distance(robotPoseVector) < robotVelocityVector.magnitude) return true
        }
        return false

    }
}
