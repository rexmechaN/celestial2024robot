package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.teamcelestial.util.CelestialSubsystem
import com.teamcelestial.util.JoystickObject
import edu.wpi.first.math.filter.SlewRateLimiter

object Arm: CelestialSubsystem() {
    private val leftArm = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val rightArm = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)

    private val leftLimiter = SlewRateLimiter(0.3)
    private val rightLimiter = SlewRateLimiter(0.3)

    override fun tick() {
        val value =
            if(JoystickObject.singleton.getRawButton(9))
                0.2
            else if(JoystickObject.singleton.getRawButton(7))
                -0.2
            else 0.0

        leftArm.set(leftLimiter.calculate(value))
        rightArm.set(rightLimiter.calculate(-value))
    }
}