package com.teamcelestial.math.solver

import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.sign

class NumericalSolver(
    private val range: IntRange,
    private val step: Double,
    private val test: ((Double) -> Double),
) {
    fun solveFor(target: Double, solverMode: NumericalSolverMode, toleranceRate: Double = 0.0): SolverResult {
        var current: Double = range.first.toDouble()
        var error: Double = Double.MAX_VALUE
        var best: Double = current
        var bestValue: Double = Double.MAX_VALUE
        val tolerance = target * toleranceRate
        println("target: $target")
        while (current <= range.last) {
            test(current).let {value ->
                println("test: $value")
                (value - target).let {
                    if(solverMode == NumericalSolverMode.A_PLUS_PARABOLIC_MINIMUM && abs(error) < abs(it)) return SolverResult(best, bestValue)
                    val toleratedError = if(error != Double.MAX_VALUE) { error - toleranceRate } else Double.MAX_VALUE
                    if(solverMode == NumericalSolverMode.A_PLUS_PARABOLIC_MINIMUM && toleratedError != Double.MAX_VALUE && toleratedError < 0 && it - tolerance <= 0 && abs(it) < abs(error)) {
                        best = current
                        bestValue = value
                        error = it
                        println("New PARABOLIC best value: $it, previous error: $error")
                    }
                    if(error != Double.MAX_VALUE && solverMode == NumericalSolverMode.A_PLUS_PARABOLIC_MINIMUM && toleratedError < 0 &&  abs(it) < abs(error)) {
                        println("A_PLUS_PARABOLIC_MINIMUM_best_diff")
                        return SolverResult(best, bestValue)

                    }
                    if(solverMode == NumericalSolverMode.A_PLUS_PARABOLIC_MAXIMUM && toleratedError != Double.MAX_VALUE && toleratedError < 0 && it - tolerance >= 0) return SolverResult(current, value)
                    if (it.absoluteValue < error.absoluteValue) {
                        best = current
                        bestValue = value
                        error = it
                        println("New best value: $it, previous error: $error")
                    }
                }
            }
            current += step
        }
        println("Returning previous value: $bestValue, previous error: $error")
        return SolverResult(best, bestValue)
    }
}

enum class NumericalSolverMode {
    A_PLUS_PARABOLIC_MINIMUM,
    A_PLUS_PARABOLIC_MAXIMUM,
    BEST_RESULT,
}