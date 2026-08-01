package frc.robot.utils

import kotlin.math.pow

class Polynomial(vararg val coefficients: Double) {
    fun calculate(value: Double): Double {
        var returnValue = 0.0
        for (i: Int in 0..<coefficients.size) {
            returnValue += coefficients[i] * (value.pow(coefficients.size - i - 1))
        }
        return returnValue
    }

    fun calculate(value: Int): Double = calculate(value.toDouble())
}
