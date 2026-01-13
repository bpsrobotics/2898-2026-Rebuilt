package frc.robot.engine

interface ObjectSensor {
    var stick: Boolean

    /** Returns whether the sensor detects an object */
    fun get(): Boolean

    /**
     * Returns true if this is the first frame the sensor has seen an object, and will wait to
     * return true until the object exits the sensors range
     */
    fun justEntered(): Boolean {
        if (!get()) {
            stick = false
            return false
        }
        if (!stick) {
            stick = true
            return true
        }
        return false
    }
}

/*class LaserSharkObjectSensor(
    channel: Int,
    var detectionRange: DistanceUnit,
    val debouncer: Debouncer = Debouncer(0.1),
) : ObjectSensor {
    override var stick: Boolean = false
    private val laserShark = Lasershark(channel)

    override fun get(): Boolean {
        return debouncer.calculate(laserShark.distanceMeters < detectionRange.asMeters)
    }
}*/
