package com.teamcelestial

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.lang.Double.max
import java.lang.Double.min
import kotlin.math.*

class Shooter(val pidControllers: List<SparkMaxPIDController>) {
    val flywheelRadius = 0.0504
    var targetRpm = 0.0
    var targetTheta = 0.0
    var startTime = 0L
    var reference = -1.0
    var lastRpmPublish = 0L
    val ballWeight = 0.230
    val defaultRpm = 5000.0
    fun start(distance: Double, height: Double, thetaInDegrees: Double) {
        startTime = System.currentTimeMillis()
        targetRpm = 5000.0/*min(max((calculateRpm(distance, height, thetaInDegrees) * 1.1).also {
            println("Calculated RPM: $it")
        }, 0.0), 5500.0)*/
        targetTheta = calculateThetaForDistance(distance, height, targetRpm).also {
            println("Calculated Theta: $it")
        }
    }

    fun stop() {
        targetRpm = 0.0
        setMotor(targetRpm)
    }

    fun tick() {
        val millis = System.currentTimeMillis() - startTime
        val target: Double = if(millis < 3000) {
            targetRpm * 0.33
        } else {
            targetRpm
        }
        setMotor(target)
        if (lastRpmPublish + 500 < System.currentTimeMillis()) {
            lastRpmPublish = System.currentTimeMillis()
            println("Target RPM: $targetRpm")
            println("Target Theta: $targetRpm")
        }
    }

    fun calculateAirResistanceMinusV(speed: Double, time: Double): Double {
        return (speed.pow(2) * 0.01208955 / ballWeight) * time
    }

    private fun calculateRpmForVelocity(velocity: Double): Double {
        return calculateRpmForEnergyTarget(0.5 * ballWeight * velocity.pow(2))
    }

    private val churroInertia = 4 * 0.0001
    private val inertiaDisc1 = 2.125 * 0.0001
    private val inertiaDisc2 = 1.5 * 0.0001

    private val totalInertia = (2 * churroInertia) + (2 * inertiaDisc1) + (2 * inertiaDisc2)

    private val rateOfRpmRetention = 0.8
    private fun calculateRpmForEnergyTarget(energyTarget: Double): Double {
        // Energy is not fully transferred from the flywheel, rateOfRpmRetention is the rate of RPM conserved
        return sqrt((2 * energyTarget) / (totalInertia * (1 - rateOfRpmRetention))) * 60 / (2 * Math.PI)
    }

    fun setMotor(rpm: Double) {
        if (reference == rpm) return
        reference = rpm
        println("Set motor: $rpm")
        pidControllers.forEach {
            it.setReference(rpm, CANSparkMax.ControlType.kVelocity)
        }
    }

    fun calculateThetaForDistance(distance: Double, height: Double, rpm: Double): Double {
        val g = 9.81
        val v = defaultRpm * 2 * Math.PI / 60
        TODO()
        val theta = atan((g * distance.pow(2) + height * v.pow(2)))
        return Math.toDegrees(theta)
    }

    fun calculateRpm(distance: Double, height: Double, thetaInDegrees: Double): Double {
        val g = 9.81
        val theta = Math.toRadians(thetaInDegrees)
        val v =  (distance / cos(theta)) * sqrt((g) / (2 * (distance * tan(theta) - height)))
        val t = distance / (v * cos(theta))
        // Flywheel speed is not equal to ring speed since the contact happens for a short time
        return calculateRpmForVelocity(v + calculateAirResistanceMinusV(v, t))
    }
}