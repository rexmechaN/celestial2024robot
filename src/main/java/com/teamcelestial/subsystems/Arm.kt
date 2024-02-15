package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Arm: SubsystemBase() {
    private val leftArm = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val rightArm = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)

    private val pid: PIDController = PIDController(0.0, 0.0, 0.0)

    private val leftLimiter = SlewRateLimiter(0.3)
    private val rightLimiter = SlewRateLimiter(0.3)

    init {
        setupPid(pid)
    }

    override fun periodic() {

    }

    fun setMotors(power: Double) {
        leftArm.set(power)
        rightArm.set(-power)
    }

    fun setSetpoint(angle: Double) {

    }

    fun tick() {
        /*val value =
            if(JoystickObject.singleton.getRawButton(9))
                0.2
            else if(JoystickObject.singleton.getRawButton(7))
                -0.2
            else 0.0
*/
    }

    private fun setupPid(controller: PIDController) {
        val kP = 0.000000029
        val kI = 0.000000000
        val kD = 0.0
        val kIz = 0.0
        val kFF = 0.0
        controller.p = kP
        controller.i = kI
        controller.d = kD
        controller.setIZone(kIz)
        controller.calculate(0.0, 0.0)
    }
}