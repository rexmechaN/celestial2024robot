package com.teamcelestial.system.shooter

import com.teamcelestial.math.util.toRadians
import com.teamcelestial.subsystems.Shooter
import com.teamcelestial.subsystems.ShooterAssembly
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

class RelativeShooterTarget(
    private val x: Double,
    private val y: Double,
    private val z: Double,
    private val theta: Double,
    private val offsetHeight: Double = 0.0,
): ShooterTarget {
    override fun getTargetDistanceAndHeightPair(shooter: Shooter): Pair<Double, Double> {
        val shooterThetaRadians: Double = ShooterAssembly.getShooterAbsTheta().toRadians()
        val height = x.absoluteValue * sin(shooterThetaRadians) + z.absoluteValue * cos(shooterThetaRadians) + offsetHeight
        val distance = x.absoluteValue * cos(shooterThetaRadians) - z.absoluteValue * sin(shooterThetaRadians)
        return Pair(distance, height)
    }

    override fun toString(): String {
        return "<RelativeShooterTarget X: $x, Y: $y, Z: $z, Theta: $theta, OffsetHeight: $offsetHeight>"
    }
}