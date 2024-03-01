package com.teamcelestial.system.shooter

import com.teamcelestial.subsystems.Shooter

interface ShooterTarget {
    fun getTargetDistanceAndHeightPair(shooter: Shooter): Pair<Double, Double>
}