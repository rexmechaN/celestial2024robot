package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.teamcelestial.util.JoystickObject
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Rotator: SubsystemBase() {
    private val leftMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMotor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

    private val limiter = SlewRateLimiter(0.3)

    fun tick() {
        val value =
            if (JoystickObject.singleton.getRawButton(8)) -0.05
            else if(JoystickObject.singleton.getRawButton(10)) 0.05
            else 0.0

        leftMotor.set(value)
        rightMotor.set(value)
    }
}