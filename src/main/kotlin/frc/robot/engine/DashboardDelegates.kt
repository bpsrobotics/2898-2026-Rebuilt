package frc.robot.engine

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.reflect.KClass
import kotlin.reflect.KProperty

fun mergePaths(parent: String, prop: String): String = if (parent == "") prop else "$parent/$prop"

class DashboardBooleanDelegate(
    private val entry: NetworkTableEntry,
    private val initialValue: Boolean,
    persist: Boolean = false,
) {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): Boolean =
        entry.getBoolean(initialValue)

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
        entry.setBoolean(value)
    }

    init {
        if (persist) {
            entry.setDefaultBoolean(initialValue)
            entry.setPersistent()
        } else {
            entry.setBoolean(initialValue)
            entry.clearPersistent()
        }
    }
}

@Suppress("unused")
class DashboardBoolean(
    private val initialValue: Boolean,
    private val path: String = "",
    private val persist: Boolean = false,
) {
    operator fun provideDelegate(thisRef: Any?, property: KProperty<*>): DashboardBooleanDelegate =
        DashboardBooleanDelegate(
            SmartDashboard.getEntry(mergePaths(path, property.name)),
            initialValue,
            persist,
        )
}

class DashboardNumberDelegate(
    private val entry: NetworkTableEntry,
    private val initialValue: Double,
    persist: Boolean = false,
) {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): Double =
        entry.getDouble(initialValue)

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
        entry.setDouble(value)
    }

    init {
        if (persist) {
            entry.setDefaultDouble(initialValue)
            entry.setPersistent()
        } else {
            entry.setDouble(initialValue)
            entry.clearPersistent()
        }
    }
}

@Suppress("unused")
class DashboardNumber(
    private val initialValue: Double,
    private val path: String = "",
    private val persist: Boolean = false,
) {
    operator fun provideDelegate(thisRef: Any?, property: KProperty<*>): DashboardNumberDelegate =
        DashboardNumberDelegate(
            SmartDashboard.getEntry(mergePaths(path, property.name)),
            initialValue,
            persist,
        )
}

class DashboardStringDelegate(
    private val entry: NetworkTableEntry,
    private val initialValue: String,
    persist: Boolean = false,
) {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): String =
        entry.getString(initialValue)

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: String) {
        entry.setString(value)
    }

    init {
        if (persist) {
            entry.setDefaultString(initialValue)
            entry.setPersistent()
        } else {
            entry.setString(initialValue)
            entry.clearPersistent()
        }
    }
}

@Suppress("unused")
class DashboardString(
    private val initialValue: String,
    private val path: String = "",
    private val persist: Boolean = false,
) {
    operator fun provideDelegate(thisRef: Any?, property: KProperty<*>): DashboardStringDelegate =
        DashboardStringDelegate(
            SmartDashboard.getEntry(mergePaths(path, property.name)),
            initialValue,
            persist,
        )
}

class DashboardEnumDelegate<T : Enum<T>>(
    key: String,
    enumClass: KClass<T>,
    initialValue: T,
    persist: Boolean = false,
) {
    val chooser = SendableChooser<T>()

    init {
        for (item in enumClass.java.enumConstants) {
            chooser.addOption(item.name, item)
        }
        chooser.setDefaultOption(initialValue.name, initialValue)
        SmartDashboard.putData(key, chooser)
        if (persist) SmartDashboard.setPersistent(key) else SmartDashboard.clearPersistent(key)
    }

    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return chooser.selected
    }
}

@Suppress("unused")
class DashboardEnum<T : Enum<T>>(
    private val enumClass: KClass<T>,
    private val initialValue: T,
    private val path: String = "",
    private val persist: Boolean = false,
) {
    operator fun provideDelegate(thisRef: Any?, property: KProperty<*>) =
        DashboardEnumDelegate(mergePaths(path, property.name), enumClass, initialValue, persist)
}

class DashboardSelectDelegate<T>(
    key: String,
    values: Map<String, T>,
    initialValue: String,
    persist: Boolean = false,
) {
    val chooser = SendableChooser<T>()

    init {
        for ((key, value) in values) {
            chooser.addOption(key, value)
        }
        chooser.setDefaultOption(initialValue, values[initialValue]!!)
        if (persist) SmartDashboard.setPersistent(key) else SmartDashboard.clearPersistent(key)
    }

    operator fun getValue(thisRef: Any?, property: KProperty<*>): T = chooser.selected
}

@Suppress("unused")
class DashboardSelect<T>(
    private val values: Map<String, T>,
    private val initialValue: String,
    private val path: String = "",
    private val persist: Boolean = false,
) {
    operator fun provideDelegate(thisRef: Any?, property: KProperty<*>) =
        DashboardSelectDelegate(mergePaths(path, property.name), values, initialValue, persist)
}
