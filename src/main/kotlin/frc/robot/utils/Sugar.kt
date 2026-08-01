package frc.robot.utils

import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.round

@Suppress("unused", "MemberVisibilityCanBePrivate")
object Sugar {
    const val TAU = 2 * PI

    fun Double.radiansToDegrees(): Double {
        return times(180 / PI)
    }

    infix fun Double.eqEpsilon(other: Double) = (this - other).absoluteValue < 0.01

    infix fun Double.eqEpsilon(other: Int) = (this - other).absoluteValue < 0.01

    fun Double.within(maxError: Double, target: Double = 0.0): Boolean =
        (this - target).absoluteValue < maxError

    fun Double.degreesToRadians(): Double {
        return times(PI / 180)
    }

    fun Int.radiansToDegrees(): Double {
        return toDouble().radiansToDegrees()
    }

    fun Int.degreesToRadians(): Double {
        return toDouble().degreesToRadians()
    }

    fun Double.clamp(min: Double = 0.0, max: Double = 1.0) = this.coerceIn(min, max)

    fun angleDifference(angle1: Double, angle2: Double): Double {
        val a = angle1 - angle2
        return (a + PI).mod(2.0 * PI) - PI
    }

    fun Double.roundTo(decimalPlace: Int): Double {
        val multiplier = 10.0.pow(decimalPlace).toInt()
        return round(this * multiplier) / multiplier
    }

    fun Double.circleNormalize(): Double {
        if (this < 0) return (this % (2 * PI)) + (2 * PI)
        return this % (2 * PI)
    }
}
