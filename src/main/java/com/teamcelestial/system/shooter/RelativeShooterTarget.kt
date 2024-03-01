package com.teamcelestial.system.shooter

import com.teamcelestial.subsystems.Shooter
import kotlin.math.cos
import kotlin.math.sin

class RelativeShooterTarget(
    private val y: Double,
    private val z: Double,
    private val theta: Double,
): ShooterTarget {
    @Suppress("UNREACHABLE_CODE")
    override fun getTargetDistanceAndHeightPair(shooter: Shooter): Pair<Double, Double> {
        val shooterTheta: Double = TODO()
        val height = z * sin(shooterTheta) + y * cos(shooterTheta)
        val distance = z * cos(shooterTheta) - y * sin(shooterTheta)
        return Pair(distance, height)
    }
}