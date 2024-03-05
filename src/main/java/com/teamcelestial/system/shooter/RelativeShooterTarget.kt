package com.teamcelestial.system.shooter

import com.teamcelestial.math.util.toRadians
import com.teamcelestial.subsystems.Shooter
import com.teamcelestial.subsystems.ShooterAssembly
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class RelativeShooterTarget(
    private val y: Double,
    private val z: Double,
    private val theta: Double,
    private val offsetHeight: Double = 0.0,
): ShooterTarget {
    override fun getTargetDistanceAndHeightPair(shooter: Shooter): Pair<Double, Double> {
        val shooterThetaRadians: Double = ShooterAssembly.getShooterAbsTheta().toRadians()
        val height = z * sin(shooterThetaRadians) + y * cos(shooterThetaRadians) + offsetHeight
        val distance = z * cos(shooterThetaRadians) - y * sin(shooterThetaRadians)
        return Pair(distance, height)
    }
}