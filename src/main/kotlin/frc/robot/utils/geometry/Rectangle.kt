package frc.robot.utils.geometry

import edu.wpi.first.math.geometry.Pose2d
import kotlin.math.absoluteValue

data class Rectangle(val coordinate1: Vector2, val coordinate2: Vector2) {
    constructor(
        x: Double,
        y: Double,
        left: Double,
        down: Double,
    ) : this(Vector2(x, y), Vector2(x + left, y - down))

    val x1
        get() = coordinate1.x

    val y1
        get() = coordinate1.y

    val x2
        get() = coordinate2.x

    val y2
        get() = coordinate2.y

    val center
        get() = (coordinate1 + coordinate2) / 2.0

    val width
        get() = (x1 - x2).absoluteValue

    val height
        get() = (y1 - y2).absoluteValue

    val topRight = center + Vector2(width / 2, height / 2)
    val topLeft = center + Vector2(-width / 2, height / 2)
    val bottomRight = center + Vector2(width / 2, -height / 2)
    val bottomLeft = center + Vector2(-width / 2, -height / 2)

    override fun toString(): String {
        return "Rect: {Top left: ${coordinate1}, Bottom right: ${coordinate2}}"
    }

    fun distToCenter(other: Vector2): Vector2 {
        return other - center
    }

    fun distToCenter(other: Pose2d): Vector2 {
        return Vector2(other.x - center.x, other.y - center.y)
    }

    fun containsX(x: Double): Boolean {
        return x in coordinate1.x..coordinate2.x
    }

    fun containsX(coordinate: Vector2): Boolean {
        return containsX(coordinate.x)
    }

    fun containsX(pose: Pose2d): Boolean {
        return containsX(pose.x)
    }

    fun containsY(y: Double): Boolean {
        return y in coordinate2.y..coordinate1.y
    }

    fun containsY(coordinate: Vector2): Boolean {
        return containsY(coordinate.y)
    }

    fun containsY(pose: Pose2d): Boolean {
        return containsY(pose.y)
    }

    fun contains(x: Double, y: Double): Boolean {
        return containsX(x) && containsY(y)
    }

    fun contains(coordinate: Vector2): Boolean {
        return contains(coordinate.x, coordinate.y)
    }

    operator fun contains(pose: Pose2d): Boolean {
        return contains(pose.x, pose.y)
    }

    fun reflectHorizontally(x: Double): Rectangle {
        val coor1 = coordinate1.reflectHorizontally(x)
        val coor2 = coordinate2.reflectHorizontally(x)
        val center = (coor1.x + coor2.x) / 2
        return Rectangle(coor1.reflectHorizontally(center), coor2.reflectHorizontally(center))
    }
}
