package frc.robot.engine

import beaverlib.utils.Units.Angular.RPM
import beaverlib.utils.Units.Angular.rotations
import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class SparkWrapper(val deviceId: Int, motorType: MotorType, configurer: SparkMaxConfig.() -> Unit) :
    Sendable {
    companion object {
        val sparksWithErrors = mutableSetOf<SparkWrapper>()
    }

    private var motor: SparkMax? = null
    var initError: REVLibError
    val config = SparkMaxConfig()

    var dashboardControl = false
    var requestPower = 0.0

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    val position
        get() = (motor?.encoder?.position ?: 0.0).rotations

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    val velocity
        get() = (motor?.encoder?.velocity ?: 0.0).RPM

    init {
        config.configurer()
        val motor = SparkMax(deviceId, motorType)
        this.motor = motor
        initError =
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        if (initError != REVLibError.kOk) this.motor = null
        updateGlobalErrorState()

        SmartDashboard.putData("Motors/Spark$deviceId", this)
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun stopMotor() {
        if (dashboardControl) return
        motor?.stopMotor()
    }

    fun set(power: Double) {
        if (dashboardControl) return
        motor?.set(power)
    }

    @Suppress("MemberVisibilityCanBePrivate", "unused")
    fun setVoltage(volts: Double) {
        if (dashboardControl) return
        motor?.setVoltage(volts)
    }

    private fun updateGlobalErrorState() {
        val haveError =
            initError != REVLibError.kOk ||
                motor == null ||
                motor?.faults?.motorType == true ||
                motor?.faults?.can == true ||
                motor?.faults?.sensor == true ||
                motor?.faults?.firmware == true ||
                motor?.faults?.escEeprom == true ||
                motor?.faults?.gateDriver == true ||
                motor?.faults?.temperature == true ||
                motor?.faults?.other == true ||
                motor?.warnings?.escEeprom == true ||
                motor?.warnings?.sensor == true ||
                motor?.warnings?.stall == true ||
                motor?.warnings?.brownout == true ||
                motor?.warnings?.overcurrent == true ||
                motor?.warnings?.extEeprom == true ||
                motor?.warnings?.hasReset == true ||
                motor?.warnings?.other == true ||
                motor?.stickyFaults?.motorType == true ||
                motor?.stickyFaults?.can == true ||
                motor?.stickyFaults?.sensor == true ||
                motor?.stickyFaults?.firmware == true ||
                motor?.stickyFaults?.escEeprom == true ||
                motor?.stickyFaults?.gateDriver == true ||
                motor?.stickyFaults?.temperature == true ||
                motor?.stickyFaults?.other == true ||
                motor?.stickyWarnings?.escEeprom == true ||
                motor?.stickyWarnings?.sensor == true ||
                motor?.stickyWarnings?.stall == true ||
                motor?.stickyWarnings?.brownout == true ||
                motor?.stickyWarnings?.overcurrent == true ||
                motor?.stickyWarnings?.extEeprom == true ||
                motor?.stickyWarnings?.hasReset == true ||
                motor?.stickyWarnings?.other == true
        if (haveError) sparksWithErrors.add(this) else sparksWithErrors.remove(this)
        SmartDashboard.putString(
            "Motors/SparkIdsWithErrors",
            sparksWithErrors.map { it.deviceId }.joinToString(),
        )
    }

    private fun stringifyFaults(faults: SparkBase.Faults): String {
        val result = mutableListOf<String>()
        if (faults.motorType) result.add("MotorType")
        if (faults.can) result.add("CAN")
        if (faults.sensor) result.add("Sensor")
        if (faults.firmware) result.add("Firmware")
        if (faults.escEeprom) result.add("Esc EEPROM")
        if (faults.gateDriver) result.add("GateDriver")
        if (faults.temperature) result.add("Temperature")
        if (faults.other) result.add("Other")
        return result.joinToString()
    }

    private fun stringifyWarnings(warnings: SparkBase.Warnings): String {
        val result = mutableListOf<String>()
        if (warnings.escEeprom) result.add("Esc EEPROM")
        if (warnings.sensor) result.add("Sensor")
        if (warnings.stall) result.add("Stall")
        if (warnings.brownout) result.add("Brownout")
        if (warnings.overcurrent) result.add("Overcurrent")
        if (warnings.extEeprom) result.add("Ext EEPROM")
        if (warnings.hasReset) result.add("Reset")
        if (warnings.other) result.add("Other")
        return result.joinToString()
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("SparkWrapper")
        builder.addDoubleProperty("position", { motor?.encoder?.position ?: Double.NaN }, null)
        builder.addDoubleProperty("velocity", { motor?.encoder?.velocity ?: Double.NaN }, null)
        builder.addDoubleProperty("busVoltage", { motor?.busVoltage ?: Double.NaN }, null)
        builder.addDoubleProperty("outputCurrent", { motor?.outputCurrent ?: Double.NaN }, null)
        builder.addIntegerProperty("id", { deviceId.toLong() }, null)
        builder.addStringProperty("initError", { initError.name }, null)
        builder.addStringProperty(
            "faults",
            { motor?.faults?.let { stringifyFaults(it) } ?: "NotPresent" },
            null,
        )
        builder.addStringProperty(
            "stickyFaults",
            { motor?.stickyFaults?.let { stringifyFaults(it) } ?: "NotPresent" },
            null,
        )
        builder.addStringProperty(
            "warnings",
            { motor?.warnings?.let { stringifyWarnings(it) } },
            null,
        )
        builder.addStringProperty(
            "stickyWarnings",
            { motor?.stickyWarnings?.let { stringifyWarnings(it) } },
            null,
        )
        builder.addBooleanProperty(
            "dashboardControl",
            { dashboardControl },
            { dashboardControl = it },
        )
        builder.addDoubleProperty(
            "requestPower",
            { requestPower },
            {
                requestPower = it
                dashboardControl = true
                motor?.set(requestPower)
            },
        )
    }
}
