package frc.robot.utils.geometry

import frc.robot.utils.asMeters
import frc.robot.utils.asRadians
import frc.robot.utils.convert
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.Units
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import kotlin.math.*

class Vector2(val x: Double, val y: Double) {
    companion object {
        @JvmName("FromNumber") fun new(x: Number, y: Number) = Vector2(x.toDouble(), y.toDouble())

        @JvmName("fromMagnitude&Angle")
        fun new(angle: Angle, magnitude: Double) =
            Vector2(cos(angle.convert(Units.Radians)) * magnitude, sin(angle.convert(Units.Radians)) * magnitude)

        @JvmName("fromDistance")
        fun new(x: Distance, y: Distance) = Vector2(x.asMeters, y.asMeters)

        @JvmName("fromVelocity")
        fun new(x: LinearVelocity, y: LinearVelocity) =
            Vector2(x.convert(Units.MetersPerSecond), y.convert(Units.MetersPerSecond))

        @JvmName("fromAcceleration")
        fun new(x: LinearAcceleration, y: LinearAcceleration) =
            Vector2(x.convert(Units.MetersPerSecondPerSecond), y.convert(Units.FeetPerSecondPerSecond))

        fun zero() = Vector2(0.0, 0.0)

        fun crossProduct(vector1: Vector2, vector2: Vector2) =
            (vector1.x * vector2.y) - (vector1.y * vector2.x)

        infix fun Vector2.dotProduct(other: Vector2) = (this.x * other.x) + (this.y * other.y)

        fun lerp(v1: Vector2, v2: Vector2, amount: Double): Vector2 {
            return v1 + ((v2 - v1) * amount)
        }
    }

    constructor(pose: Pose2d) : this(pose.x, pose.y)

    constructor(angle: Angle) : this(cos(angle.convert(Units.Radians)), sin(angle.convert(Units.Radians)))

    fun rotateBy(angle: Double): Vector2 {
        return Vector2(cos(angle) * x - sin(angle) * y, sin(angle) * x + cos(angle) * y)
    }

    fun rotateBy(angle: Angle): Vector2 {
        return rotateBy(angle.convert(Units.Radians))
    }

    val magnitude
        get() = sqrt(x.pow(2) + y.pow(2))

    val angle
        get() = Units.Radians.of(atan2(y, x))

    val unit
        get() = this / this.magnitude

    fun angleTo(other: Vector2) = (this - other).angle

    fun distance(other: Vector2): Double = (this - other).magnitude

    fun distance(pose: Pose2d): Double = distance(Vector2(pose))

    fun xdistance(pos: Double): Double = x - pos

    fun xdistance(pos: Vector2): Double = x - pos.x

    fun xdistance(pos: Pose2d): Double = x - pos.x

    fun ydistance(pos: Double): Double = y - pos

    fun ydistance(pos: Vector2): Double = y - pos.y

    fun ydistance(pos: Pose2d): Double = y - pos.y

    operator fun plus(other: Vector2) = Vector2(x + other.x, y + other.y)

    operator fun minus(other: Vector2) = Vector2(x - other.x, y - other.y)

    operator fun plus(other: Pose2d) = this + other.vector2

    operator fun minus(other: Pose2d) = this - other.vector2

    operator fun times(other: Double) = Vector2(x * other, y * other)

    operator fun div(other: Double) = Vector2(x / other, y / other)

    operator fun unaryMinus() = Vector2(-x, -y)

    operator fun unaryPlus() = this

    override fun toString() = "(x: ${x}, y: ${y})"

    fun reflectHorizontally(x: Double) = Vector2(x + (x - this.x), y)

    fun toPose2d(rotation: Angle) = Pose2d(x, y, Rotation2d.fromRadians(rotation.convert(Units.Radians)))
}

val Pose2d.vector2
    get() = Vector2(this)
val ChassisSpeeds.vector2
    get() = Vector2(this.vxMetersPerSecond, this.vyMetersPerSecond)
