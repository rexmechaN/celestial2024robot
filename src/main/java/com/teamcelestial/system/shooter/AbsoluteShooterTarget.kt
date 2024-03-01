package com.teamcelestial.system.shooter

import com.teamcelestial.subsystems.Shooter

class AbsoluteShooterTarget(
    private val distance: Double,
    private val height: Double
): ShooterTarget {
    override fun getTargetDistanceAndHeightPair(shooter: Shooter): Pair<Double, Double> {
        return Pair(distance, height)
    }
}