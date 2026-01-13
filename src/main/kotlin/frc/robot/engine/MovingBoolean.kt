package frc.robot.engine

import kotlin.reflect.KProperty

class MovingBoolean(size: Int, var threshold: Int) {
    private val buffer = BooleanArray(size)
    private var i = 0

    fun add(v: Boolean) {
        buffer[i++ % buffer.size] = v
    }

    val numberTrue: Int
        get() = buffer.filter({ b -> b }).size

    val percentTrue: Double
        get() = buffer.size.toDouble() / numberTrue

    inline operator fun getValue(thisRef: Any?, property: KProperty<*>): Boolean =
        numberTrue >= threshold
}
