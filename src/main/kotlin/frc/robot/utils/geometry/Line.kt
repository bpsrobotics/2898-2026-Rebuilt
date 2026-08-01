package frc.robot.utils.geometry

import frc.robot.utils.Sugar.eqEpsilon
import frc.robot.utils.asRadians
import frc.robot.utils.getCoterminal
import frc.robot.utils.geometry.Vector2.Companion.dotProduct
import edu.wpi.first.math.geometry.Pose2d
import kotlin.math.*

class Line(val point1: Vector2, val point2: Vector2) {
    val slope
        get() = (point2.x - point1.x) / (point2.y - point1.y)

    fun intersects(raycast: Raycast2D): Boolean {
        val theta1 = raycast.origin.angleTo(point1).getCoterminal()
        val theta2 = raycast.origin.angleTo(point2).getCoterminal()
        return min(theta1.asRadians, theta2.asRadians) <= raycast.angle.asRadians &&
            raycast.angle.asRadians <= max(theta1.asRadians, theta2.asRadians)
    }

    fun distance(raycast: Raycast2D): Double? {
        val rayDirection = Vector2(raycast.angle)
        val v1 = raycast.origin - point1
        val v2 = point2 - point1
        val v3 = Vector2(-rayDirection.y, rayDirection.x)

        val dot = v2 dotProduct v3
        if (dot eqEpsilon 0) return null

        val t1 = Vector2.crossProduct(v2, v1) / dot
        val t2 = v1 dotProduct v3 / dot

        return if (t1 >= 0.0 && t2 >= 0.0 && t2 <= 1.0) t1 else null
    }

    val center
        get() = (point1 + point2) / 2.0

    fun distance(pose: Pose2d): Double? {
        return distance(Raycast2D(pose))
    }

    fun intersection(raycast: Raycast2D): Vector2? {
        val intersectionDistance = distance(raycast) ?: return null
        return raycast.origin + Vector2(raycast.angle) * intersectionDistance
    }

    fun intersection(pose: Pose2d): Vector2? {
        return intersection(Raycast2D(pose))
    }

    fun reflectHorizontally(x: Double): Line {
        return Line(point1.reflectHorizontally(x), point2.reflectHorizontally(x))
    }
}
