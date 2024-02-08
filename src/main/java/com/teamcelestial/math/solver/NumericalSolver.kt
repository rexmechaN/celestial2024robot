package com.teamcelestial.math.solver

import kotlin.math.absoluteValue

class NumericalSolver(
    val range: IntRange,
    val step: Double,
    val test: ((Double) -> Double),
) {
    fun solveFor(target: Double): SolverResult {
        var current: Double = range.first.toDouble()
        var error: Double = Double.MAX_VALUE
        var best: Double = current
        var bestValue: Double = Double.MAX_VALUE
        while (current <= range.last) {
            test(current).let {value ->
                (value - target).absoluteValue.let {
                    if (it < error) {
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