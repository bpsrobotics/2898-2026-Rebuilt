package frc.robot.utils.geometry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle

class Raycast2D(val origin: Vector2, val angle: Angle) {
    constructor(pose: Pose2d) : this(Vector2(pose), Units.Radians.of(pose.rotation.radians))
}
