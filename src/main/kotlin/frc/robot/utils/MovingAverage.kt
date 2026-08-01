package frc.robot.utils

import kotlin.math.sign
import kotlin.reflect.KProperty

@Suppress("NOTHING_TO_INLINE")
class MovingAverage(val size: Int) {
    private val buffer = DoubleArray(size)
    private var i = 0

    var average = 0.0
        private set

    val median
        get() = buffer.sorted()[buffer.size / 2]

    fun add(v: Double): Double {
        buffer[i++ % buffer.size] = v
        average = buffer.average()
        return average
    }

    fun clear(v: Double = 0.0) {
        buffer.fill(v)
    }

    inline operator fun getValue(thisRef: Any?, property: KProperty<*>): Double = average

    inline operator fun invoke(v: Double): Double {
        return add(v)
    }

    inline operator fun compareTo(other: Number): Int {
        return sign(average - other.toDouble()).toInt()
    }
}
