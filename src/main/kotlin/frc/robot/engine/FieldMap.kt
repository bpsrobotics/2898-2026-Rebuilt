package frc.robot.engine

import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.feet
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.geometry.Rectangle
import beaverlib.utils.geometry.Vector2
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.sqrt

interface FeederStation {
    val center: Vector2
    val shape: Rectangle
}

interface Zoo {
    val center: Vector2
    val shape: Rectangle
}

object FieldMap {
    val FieldLength = 54.feet + 8.75.inches
    val FieldHeight = 26.feet + 4.inches
    val FeederWidth = 20.inches
    val ZooWidth = 96.inches
    val FieldCenter = Vector2(FieldLength.asMeters / 2, FieldHeight.asMeters / 2)
    val CarrotPatchWidth = 6.feet

    fun zooSafeRadius(safeDistance: DistanceUnit): DistanceUnit {
        return (ZooWidth / 2) * sqrt(2.0) + safeDistance
    }

    object BlueFeederStation : FeederStation {
        override val center =
            Vector2(((FieldLength / 2) - 15.feet).asMeters, (FieldHeight / 2).asMeters)
        override val shape =
            Rectangle(
                center - Vector2(FeederWidth.asMeters / 2, FeederWidth.asMeters / 2),
                center + Vector2(FeederWidth.asMeters / 2, FeederWidth.asMeters / 2),
            )
    }

    object RedFeederStation : FeederStation {
        override val center =
            Vector2(((FieldLength / 2) + 15.feet).asMeters, (FieldHeight / 2).asMeters)
        override val shape =
            Rectangle(
                center - Vector2(FeederWidth.asMeters / 2, FeederWidth.asMeters / 2),
                center + Vector2(FeederWidth.asMeters / 2, FeederWidth.asMeters / 2),
            )
    }

    object BlueZoo : Zoo {
        override val center = BlueFeederStation.center
        override val shape =
            Rectangle(
                center - Vector2(ZooWidth.asMeters / 2, ZooWidth.asMeters / 2),
                center + Vector2(ZooWidth.asMeters / 2, ZooWidth.asMeters / 2),
            )
    }

    object RedZoo : Zoo {
        override val center = RedFeederStation.center
        override val shape =
            Rectangle(
                center - Vector2(ZooWidth.asMeters / 2, ZooWidth.asMeters / 2),
                center + Vector2(ZooWidth.asMeters / 2, ZooWidth.asMeters / 2),
            )
    }

    object CarrotPatch {
        val center = FieldCenter
    }

    val teamFeederStation: FeederStation
        get() =
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                RedFeederStation
            } else {
                BlueFeederStation
            }

    val teamZoo: Zoo
        get() =
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                RedZoo
            } else {
                BlueZoo
            }
}
