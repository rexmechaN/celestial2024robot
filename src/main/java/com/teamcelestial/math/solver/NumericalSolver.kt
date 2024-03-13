package com.teamcelestial.math.solver

import kotlin.math.absoluteValue
import kotlin.math.sign

class NumericalSolver(
    private val range: IntRange,
    private val step: Double,
    private val test: ((Double) -> Double),
) {
    fun solveFor(target: Double, solverMode: NumericalSolverMode): SolverResult {
        var current: Double = range.first.toDouble()
        var error: Double = Double.MAX_VALUE
        var best: Double = current
        var bestValue: Double = Double.MAX_VALUE
        while (current <= range.last) {
            test(current).let {value ->
                (value - target).let {
                    if(solverMode == NumericalSolverMode.A_PLUS_PARABOLIC_MINIMUM && error != Double.MAX_VALUE && error > 0 && it <= 0) return SolverResult(current, value)
                    if(solverMode == NumericalSolverMode.A_PLUS_PARABOLIC_MAXIMUM && error != Double.MAX_VALUE && error < 0 && it >= 0) return SolverResult(current, value)
                    if (it.absoluteValue < error.absoluteValue) {
                        best = current
                        bestValue = value
                        error = it
                    }
                }
            }
            current += step
        }
        return SolverResult(best, bestValue)
    }
}

enum class NumericalSolverMode {
    A_PLUS_PARABOLIC_MINIMUM,
    A_PLUS_PARABOLIC_MAXIMUM,
    BEST_RESULT,
}