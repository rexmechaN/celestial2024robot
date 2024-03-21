package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake: SubsystemBase() {
    private val motor = CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless)

    fun setMotor(power: Double) {
        motor.set(power)
    }
}