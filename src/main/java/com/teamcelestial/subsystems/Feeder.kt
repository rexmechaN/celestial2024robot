package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Feeder: SubsystemBase() {
    private val feeder = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)

    fun setMotor(power: Double) {
        feeder.set(power)
    }
}