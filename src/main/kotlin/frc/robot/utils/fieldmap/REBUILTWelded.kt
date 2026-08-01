package frc.robot.utils.fieldmap

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.utils.asMeters
import frc.robot.utils.feet
import frc.robot.utils.geometry.Line
import frc.robot.utils.geometry.Rectangle
import frc.robot.utils.geometry.Vector2
import frc.robot.utils.inches

interface Hub {
    val center: Vector2
    val shape: Rectangle
}

interface Trench {
    val centerX: Double
    val line: Line
}

object FieldMapREBUILTWelded {
    val FieldLength = 54.feet + 8.75.inches
    val FieldHeight = 26.feet + 4.inches
    val HubWidth = 46.508.inches.asMeters

    object BlueHub : Hub {
        override val center = Vector2(182.11.inches.asMeters, 158.84.inches.asMeters)
        override val shape =
            Rectangle(
                center - Vector2(HubWidth, HubWidth) / 2.0,
                center + Vector2(HubWidth, HubWidth) / 2.0,
            )
    }

    object RedHub : Hub {
        override val center =
            Vector2(651.22.inches.asMeters - 182.11.inches.asMeters, 158.84.inches.asMeters)
        override val shape =
            Rectangle(
                center - Vector2(HubWidth, HubWidth) / 2.0,
                center + Vector2(HubWidth, HubWidth) / 2.0,
            )
    }

    val teamHub
        get() =
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                RedHub
            } else {
                BlueHub
            }

    val trenches = arrayOf(TopRedTrench, BottomRedTrench, TopBlueTrench, BottomBlueTrench)

    enum class TrenchPos {
        Bottom,
        Top,
    }

    fun teamTrenches(
        team: DriverStation.Alliance = DriverStation.getAlliance().get()
    ): Map<TrenchPos, Trench> =
        if (team == DriverStation.Alliance.Red) {
            mapOf(Pair(TrenchPos.Bottom, BottomRedTrench), Pair(TrenchPos.Top, TopRedTrench))
        } else {
            mapOf(Pair(TrenchPos.Bottom, BottomBlueTrench), Pair(TrenchPos.Top, BottomBlueTrench))
        }

    val BlueAllianceAreaLineX = 182.11.inches
    val RedAllianceAreaLineX = FieldLength - 182.11.inches

    object TopBlueTrench : Trench {
        override val centerX: Double = 182.11.inches.asMeters
        override val line: Line =
            Line(
                Vector2(centerX, 317.69.inches.asMeters),
                Vector2(centerX, (317.69 - 50.59).inches.asMeters),
            )
        val shape: Rectangle
            get() = TODO("Not yet implemented")
    }

    object BottomBlueTrench : Trench {
        override val centerX: Double = 182.11.inches.asMeters
        override val line: Line =
            Line(Vector2(centerX, 0.0.inches.asMeters), Vector2(centerX, 50.59.inches.asMeters))
        val shape: Rectangle
            get() = TODO("Not yet implemented")
    }

    object TopRedTrench : Trench {
        override val centerX: Double = 469.11.inches.asMeters
        override val line: Line =
            Line(
                Vector2(centerX, 317.69.inches.asMeters),
                Vector2(centerX, (317.69 - 50.59).inches.asMeters),
            )
        val shape: Rectangle
            get() = TODO("Not yet implemented")
    }

    object BottomRedTrench : Trench {
        override val centerX: Double = 469.11.inches.asMeters
        override val line: Line =
            Line(Vector2(centerX, 0.0.inches.asMeters), Vector2(centerX, 50.59.inches.asMeters))
        val shape: Rectangle
            get() = TODO("Not yet implemented")
    }

    enum class AllianceArea() {
        Red,
        RedTrench,
        Neutral,
        BlueTrench,
        Blue,
    }

    fun getPoseAllianceArea(pose: Pose2d): AllianceArea {
        return when {
            (pose.x < BlueHub.shape.bottomLeft.x) -> AllianceArea.Blue
            (pose.x < BlueHub.shape.bottomRight.x) -> AllianceArea.BlueTrench
            (pose.x < RedHub.shape.bottomLeft.x) -> AllianceArea.Neutral
            (pose.x < RedHub.shape.bottomRight.x) -> AllianceArea.RedTrench
            else -> AllianceArea.Red
        }
    }

    fun getTeamAllianceArea(
        alliance: DriverStation.Alliance = DriverStation.getAlliance().get()
    ): AllianceArea =
        if (alliance == DriverStation.Alliance.Red) {
            AllianceArea.Red
        } else {
            AllianceArea.Blue
        }
}
