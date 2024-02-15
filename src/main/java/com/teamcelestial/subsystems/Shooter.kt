package com.teamcelestial.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase

import kotlin.math.*

object Shooter: SubsystemBase() {
    private val leftCim = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless)
    private val rightCim = CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless)

    private val leftPid = leftCim.pidController
    private val rightPid = rightCim.pidController
    private val pidControllers = listOf(leftPid, rightPid)

    init {
        setupPid(leftPid)
        setupPid(rightPid)
    }

    private val flywheelRadius = 0.0504
    private var targetRpm = 0.0
    private var targetTheta = 0.0
    private var startTime = 0L
    private var reference = -1.0
    private var lastRpmPublish = 0L
    private val ballWeight = 0.230

    private var shooterStarted = false

    fun setMotors(power: Double) {
        leftCim.set(power)
        rightCim.set(power)
    }

    fun tick() {
        trackTriggers()

        if (lastRpmPublish + 500 < System.currentTimeMillis()) {
            lastRpmPublish = System.currentTimeMillis()
        }
        val millis = System.currentTimeMillis() - startTime
        val target: Double = if (millis < 3000) {
            targetRpm * 0.33
        } else {
            targetRpm

        }
        setMotor(target)
        if (lastRpmPublish + 500 < System.currentTimeMillis()) {
            lastRpmPublish = System.currentTimeMillis()
        }
    }

    private fun trackTriggers() {
        /*if (JoystickObject.singleton.getRawButton(1)) {
            if (!shooterStarted) {
                start(2.65, 0.7)
                shooterStarted = true
            }
        } else {
            if (shooterStarted) {
                stop()
                shooterStarted = false
            }
        }*/
    }

    private fun start(distance: Double, height: Double) {
        startTime = System.currentTimeMillis()

        calculateRpm(distance, height, 25.0).let {
            targetRpm = it
            println("Target RPM: $targetRpm")
            println("Target Theta: $targetTheta")
        }
    }

    private fun stop() {
        targetRpm = 0.0
        setMotor(targetRpm)
    }

    private fun calculateAirResistanceMinusV(speed: Double, time: Double): Double {
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

    private fun setMotor(rpm: Double) {
        if (reference == rpm) return
        reference = rpm
        println("Set motor: $rpm")
        println("motor power ${leftCim.get()}")
        pidControllers.forEach {
            it.setReference(rpm, CANSparkBase.ControlType.kVelocity)
        }
    }

    private fun calculateRpm(distance: Double, height: Double, thetaInDegrees: Double): Double {
        val g = 9.81
        val theta = Math.toRadians(thetaInDegrees)
        val v = (distance / cos(theta)) * sqrt((g) / (2 * (distance * tan(theta) - height)))
        val t = distance / (v * cos(theta))
        // Flywheel speed is not equal to ring speed since the contact happens for a short time
        return calculateRpmForVelocity(v + calculateAirResistanceMinusV(v, t))
    }

    private fun setupPid(controller: SparkPIDController?) {
        println("P: " + controller!!.p)
        println("I: " + controller.i)
        println("D: " + controller.d)
        println("F: " + controller.ff)
        println("Iacc: " + controller.iAccum)
        println("======")
        val kP = 0.000000039
        val kI = 0.00000029
        val kD = 0.0
        val kIz = 0.0
        val kFF = 0.0
        val kMaxOutput = 1.0
        val kMinOutput = -1.0
        val maxRPM = 5700.0
        // set PID coefficients
        controller.setP(kP)
        controller.setI(kI)
        controller.setD(kD)
        controller.setIZone(kIz)
        controller.setFF(kFF)
        controller.setOutputRange(kMinOutput, kMaxOutput)
    }
}