@file:Suppress("unused")

package frc.robot.utils

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit as WpiUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Energy
import edu.wpi.first.units.measure.Force
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.Power
import edu.wpi.first.units.measure.Time
import kotlin.math.PI

fun <U : WpiUnit> Measure<U>.convert(unit: U): Double = `in`(unit)

val Distance.asMeters: Double
    get() = convert(Units.Meters)
val Angle.asRadians: Double
    get() = convert(Units.Radians)
val Time.asSeconds: Double
    get() = convert(Units.Seconds)
val Mass.asKilograms: Double
    get() = convert(Units.Kilograms)

val Rotation2d.angle: Angle
    get() = Units.Radians.of(radians)

fun Angle.getCoterminal(): Angle = Units.Radians.of((convert(Units.Radians) + 2 * PI).mod(2 * PI))

val Number.meters: Distance
    get() = Units.Meters.of(toDouble())
val Number.feet: Distance
    get() = Units.Feet.of(toDouble())
val Number.inches: Distance
    get() = Units.Inches.of(toDouble())

val Number.metersPerSecond: LinearVelocity
    get() = Units.MetersPerSecond.of(toDouble())
val Number.feetPerSecond: LinearVelocity
    get() = Units.FeetPerSecond.of(toDouble())
val Number.KPH: LinearVelocity
    get() = Units.MetersPerSecond.of(toDouble() / 3.6)
val Number.MPH: LinearVelocity
    get() = Units.MetersPerSecond.of(toDouble() * 0.44704)

val Number.metersPerSecondSquared: LinearAcceleration
    get() = Units.MetersPerSecondPerSecond.of(toDouble())
val Number.feetPerSecondSquared: LinearAcceleration
    get() = Units.FeetPerSecondPerSecond.of(toDouble())

val Number.rotations: Angle
    get() = Units.Rotations.of(toDouble())
val Number.radians: Angle
    get() = Units.Radians.of(toDouble())
val Number.degrees: Angle
    get() = Units.Degrees.of(toDouble())

val Number.radiansPerSecond: AngularVelocity
    get() = Units.RadiansPerSecond.of(toDouble())
val Number.rotationsPerSecond: AngularVelocity
    get() = Units.RotationsPerSecond.of(toDouble())
val Number.RPM: AngularVelocity
    get() = Units.RPM.of(toDouble())

val Number.radiansPerSecondSquared: AngularAcceleration
    get() = Units.RadiansPerSecondPerSecond.of(toDouble())
val Number.rotationsPerSecondSquared: AngularAcceleration
    get() = Units.RotationsPerSecondPerSecond.of(toDouble())

val Number.ms: Time
    get() = Units.Milliseconds.of(toDouble())
val Number.sec: Time
    get() = Units.Seconds.of(toDouble())
val Number.min: Time
    get() = Units.Minutes.of(toDouble())
val Number.hours: Time
    get() = Units.Seconds.of(toDouble() * 3600)
val Number.days: Time
    get() = Units.Seconds.of(toDouble() * 86400)

val Number.Hz: Frequency
    get() = Units.Hertz.of(toDouble())
val Number.mHz: Frequency
    get() = Units.Millihertz.of(toDouble())
val Number.kHz: Frequency
    get() = Units.Kilo(Units.Hertz).of(toDouble())

val Number.kg: Mass
    get() = Units.Kilograms.of(toDouble())
val Number.lbs: Mass
    get() = Units.Pounds.of(toDouble())

val Number.newtons: Force
    get() = Units.Newtons.of(toDouble())
val Number.lbF: Force
    get() = Units.PoundsForce.of(toDouble())

val Number.watts: Power
    get() = Units.Watts.of(toDouble())
val Number.milliwatts: Power
    get() = Units.Milliwatts.of(toDouble())
val Number.kilowatts: Power
    get() = Units.Kilo(Units.Watts).of(toDouble())

val Number.joules: Energy
    get() = Units.Joules.of(toDouble())
val Number.millijoules: Energy
    get() = Units.Millijoules.of(toDouble())
val Number.kilojoules: Energy
    get() = Units.Kilojoules.of(toDouble())
