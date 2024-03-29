package com.teamcelestial

import com.revrobotics.CANSparkBase
import com.revrobotics.SparkPIDController
import com.teamcelestial.math.solver.NumericalSolver
import com.teamcelestial.math.solver.NumericalSolverMode
import com.teamcelestial.math.util.toRadians
import com.teamcelestial.system.shooter.ShooterCalcResult
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import kotlin.math.*

fun main() {
    start(1.5, 2.09, thetaOverride = 30.0)
    val feedforward = ArmFeedforward(0.0, 0.0,0.0, 0.0)
}



private val flywheelRadius = 0.0504
private var targetRpm = 0.0
private var targetTheta = 0.0
private var startTime = 0L
private var reference = -1.0
private var lastRpmPublish = 0L
private val ballWeight = 0.230
private val ballFinalSpeedTarget = 0.2

private var airResistanceMultiplier = 0.7
private var distRpm = 650.0

fun start(
    distance: Double,
    height: Double,
    runMotors: Boolean = true,
    thetaOverride: Double? = null,
    rpmOverride: Double? = null,
): ShooterCalcResult {
    startTime = System.currentTimeMillis()

    NumericalSolver(
        5..89,
        0.5
    ) {
        calculateRpm(distance, height, it)
    }.solveFor(min(4800.0, thetaOverride?.let {
        calculateRpm(distance, height, it)
    } ?: (4000.0 + distance * distRpm).also {
        println("RPM target $it")
    }).let {
           if(it.isNaN()) {
               5000.0
           } else it
    }, solverMode = NumericalSolverMode.A_PLUS_PARABOLIC_MINIMUM, toleranceRate = 0.1).let {
        if (runMotors) targetRpm = rpmOverride ?: it.y
        if (runMotors) targetTheta = it.x
        return ShooterCalcResult(rpm = it.y, theta = it.x).also { shooterCalcResult ->
            println(shooterCalcResult)
        }
    }
}

private fun calculateApproximateRpmTargetWithoutFlightData(hVelocity: Double, height: Double): Double {
    return calculateRpmForEnergyTarget((0.5 * ballWeight * (hVelocity.pow(2) + ballFinalSpeedTarget.pow(2)) + ballWeight * 9.81 * height).also {
        println("Energy Target: $it")
    } * 1.1).also {
        println("RPM Target: $it")
    }
}

fun stop() {
    targetRpm = 0.0
}

private fun calculateAirResistanceMinusV(speed: Double, time: Double): Double {
    return (speed.pow(2) * airResistanceMultiplier * 0.01208955 / ballWeight) * time
}

private fun calculateRpmForVelocity(velocity: Double): Double {
    return calculateRpmForEnergyTarget(0.5 * ballWeight * velocity.pow(2))
}

private val churroInertia = 4 * 0.0001
private val inertiaDisc1 = 2.125 * 0.0001
private val inertiaDisc2 = 1.5 * 0.0001

private var totalInertia = ((2 * churroInertia) + (6 * inertiaDisc1) + (6 * inertiaDisc2)) * 1.0

private val rateOfRpmRetention = 0.7
private fun calculateRpmForEnergyTarget(energyTarget: Double): Double {
    // Energy is not fully transferred from the flywheel, rateOfRpmRetention is the rate of RPM conserved
    return sqrt((2 * energyTarget) / (totalInertia * (1 - rateOfRpmRetention))) * 60 / (2 * Math.PI)
}

private fun calculateRpm(distance: Double, height: Double, thetaInDegrees: Double): Double {
    val g = 9.81
    val theta = thetaInDegrees.toRadians()
    val v = (distance / cos(theta)) * sqrt((g) / (2 * (distance * tan(theta) - height)))
    val t = distance / (v * cos(theta))
    // Flywheel speed is not equal to ring speed since the contact happens for a short time
    return calculateRpmForVelocity(v + calculateAirResistanceMinusV(v, t)).also {
        println("RPM for velocity $v: $it, Theta: $thetaInDegrees")
    }
}