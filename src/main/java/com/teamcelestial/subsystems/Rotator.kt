package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Rotator: SubsystemBase() {
    private val leftMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMotor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

    private val limiter = SlewRateLimiter(0.3)

    fun setMotors(power: Double) {
        leftMotor.set(power)
        rightMotor.set(power)
    }

    fun tick() {
    }
}