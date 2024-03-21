package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Feeder: SubsystemBase() {
    private val motor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)

    private val distanceSensor = Ultrasonic(2, 3)
    private val distanceToleranceMM = 0.0 //TODO: Set distance tolerance

    init {
        distanceSensor.isEnabled = true
    }

    override fun periodic() {
        distanceSensor.ping()
        SmartDashboard.putNumber("Distance Sensor", distanceSensor.rangeMM)
    }

    fun setMotor(power: Double, isTakingNote: Boolean = false) {
        /*val reachedTolerance = distanceSensor.rangeMM <= distanceToleranceMM
        if(isTakingNote && reachedTolerance)
            motor.set(0.0)
        else
            motor.set(power)*/
        motor.set(power)
    }
}