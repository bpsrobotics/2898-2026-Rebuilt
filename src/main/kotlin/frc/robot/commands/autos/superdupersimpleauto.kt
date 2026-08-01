package frc.robot.commands.autos

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.VisionTurningHandler
import frc.robot.utils.RPM
import frc.robot.utils.asMeters
import frc.robot.utils.asRadians
import frc.robot.utils.convert
import frc.robot.utils.fieldmap.FieldMapREBUILTWelded
import frc.robot.utils.geometry.vector2
import frc.robot.utils.meters
import frc.robot.utils.radians
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

private fun Angle.standardPosition(): Angle =
    Units.Radians.of((convert(Units.Radians) + 2 * PI).mod(2 * PI))

class MoveDistanceAndRotate(private val desiredDistance: Distance = 2.meters) : Command() {
    private val distancePID = PIDController(2.0, 0.3, 0.1)

    private val rotationPID = PIDController(2.0, 0.01, 0.2)

    init {
        rotationPID.enableContinuousInput(-PI, PI)
        addRequirements(Drivetrain)
    }

    override fun execute() {
        val target = FieldMapREBUILTWelded.teamHub.center
        val currentAngleToCenter =
            (Drivetrain.pose.vector2.angleTo(target) + PI.radians).standardPosition()
        val distanceSpeed =
            distancePID.calculate(
                Drivetrain.pose.vector2.distance(target) - desiredDistance.asMeters
            )
        rotationPID.setpoint = currentAngleToCenter.asRadians

        val speeds =
            ChassisSpeeds(
                cos(currentAngleToCenter.convert(Units.Radians)) * distanceSpeed,
                sin(currentAngleToCenter.convert(Units.Radians)) * distanceSpeed,
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
        .alongWith(Shooter.Hood.resetCommand())
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
