package com.teamcelestial

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxPIDController
import java.lang.Double.max
import java.lang.Double.min
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

class Shooter(val pidControllers: List<SparkMaxPIDController>) {
    val flywheelRadius = 0.0504
    var targetRpm = 0.0
    var startTime = 0L
    var reference = -1.0
    var lastRpmPublish = 0L
    val ballWeight = 0.230
    fun start(distance: Double, height: Double, thetaInDegrees: Double) {
        startTime = System.currentTimeMillis()
        targetRpm = min(max((calculateRpm(distance, height, thetaInDegrees) * 1.1).also {
            println("Calculated RPM: $it")
        }, 0.0), 5500.0)
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
        }
    }

    fun calculateAirResistanceMinusV(speed: Double, time: Double): Double {
        return (speed.pow(2) * 0.01208955 / ballWeight) * time
    }

    fun setMotor(rpm: Double) {
        if (reference == rpm) return
        reference = rpm
        println("Set motor: $rpm")
        pidControllers.forEach {
            it.setReference(rpm, CANSparkMax.ControlType.kVelocity)
        }
    }

    fun calculateRpm(distance: Double, height: Double, thetaInDegrees: Double): Double {
        val g = 9.81
        val theta = Math.toRadians(thetaInDegrees)
        val v =  (distance / cos(theta)) * sqrt((g) / (2 * (distance * tan(theta) - height)))
        val t = distance / (v * cos(theta))
        // Flywheel speed is not equal to ring speed since the contact happens for a short time
        return ((v + calculateAirResistanceMinusV(v, t)) / (2 * Math.PI * flywheelRadius)) * 60
    }
}