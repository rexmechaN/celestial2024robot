package com.teamcelestial.system.shooter

import com.teamcelestial.math.util.toRadians
import com.teamcelestial.subsystems.Shooter
import com.teamcelestial.subsystems.ShooterAssembly
import kotlin.math.absoluteValue
import kotlin.math.sin

const val LINK1_LENGTH = 0.30
const val LINK2_LENGTH = 0.10
const val BASE_HEIGHT = 0.20
class AbsoluteShooterTarget(
    private val distance: Double,
    private val height: Double
): ShooterTarget {
    override fun getTargetDistanceAndHeightPair(shooter: Shooter): Pair<Double, Double> {
        val shooterThetaRadians: Double = ShooterAssembly.getShooterAbsTheta().toRadians()
        val link1ThetaRadians = ShooterAssembly.getLink1Theta().toRadians()
        val link1AbsHeight = LINK1_LENGTH * sin(link1ThetaRadians).absoluteValue
        val link2AbsHeight = LINK2_LENGTH * sin(shooterThetaRadians).absoluteValue
        return Pair(distance, height - (link1AbsHeight + link2AbsHeight + BASE_HEIGHT))
    }

    override fun toString(): String {
        return "<AbsoluteShooterTarget Distance: $distance, Height: $height>"
    }
}
