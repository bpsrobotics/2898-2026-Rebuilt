package frc.robot.commands.autos

import beaverlib.fieldmap.FieldMapREBUILTWelded
import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.meters
import beaverlib.utils.geometry.vector2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.VisionTurningHandler
import kotlin.math.PI

class MoveDistanceAndRotate(private val desiredDistance: DistanceUnit = 1.meters) : Command() {
    private val distancePID = PIDController(2.0, 0.3, 0.1)

    private val rotationPID = PIDController(2.0, 0.01, 0.2)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
        addRequirements(Drivetrain)
    }

    override fun execute() {
        val target = FieldMapREBUILTWelded.teamHub.center
        val currentAngleToCenter = Drivetrain.pose.vector2.angleTo(target)
        val distanceSpeed =
            distancePID.calculate(
                Drivetrain.pose.vector2.distance(target) - desiredDistance.asMeters
            )
        rotationPID.setpoint =
            Drivetrain.pose.vector2.angleTo(FieldMapREBUILTWelded.teamHub.center).asRadians

        val speeds =
            ChassisSpeeds(
                currentAngleToCenter.cos() * distanceSpeed,
                currentAngleToCenter.sin() * distanceSpeed,
                rotationPID.calculate(Drivetrain.pose.rotation.radians),
            )
        Drivetrain.driveFieldOriented(speeds)
    }

    override fun isFinished(): Boolean {
        return distancePID.atSetpoint() && rotationPID.atSetpoint()
    }
}

fun superdupersimpleauto(): Command {
    return MoveDistanceAndRotate()
        .deadlineFor(Shooter.runAtSpeed({ 4500.RPM }))
        .andThen(
            Shooter.Hood.moveToPosition { VisionTurningHandler.goalHoodAngle }
                .deadlineFor(Shooter.runAtSpeed({ 4500.RPM }))
        )
        .andThen(
            Shooter.runAtSpeed()
                .until { Shooter.atSpeed }
                .deadlineFor(Shooter.Hood.holdPosition { VisionTurningHandler.goalHoodAngle })
        )
        .andThen(
            Shooter.Feeder.getJiggyWithIt(1.0)
                .withTimeout(10.0)
                .alongWith(Shooter.Hood.holdPosition { VisionTurningHandler.goalHoodAngle })
        )
}
