package frc.robot.engine

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

/**
 * @param`deviceId the device ID
 */
class HedgieSparkMax(deviceId : Int, type: SparkLowLevel.MotorType) : SparkMax(deviceId, type), Sendable {
    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("SparkMax")
        builder.addDoubleProperty("position", this.encoder::getPosition, null)
        builder.addDoubleProperty("velocity", this.encoder::getVelocity, null)
        builder.addDoubleProperty("current", this::getOutputCurrent, null)
        builder.addDoubleProperty("temperature", this::getMotorTemperature, null)
        builder.addDoubleProperty("power", this::get, null)
    }
}