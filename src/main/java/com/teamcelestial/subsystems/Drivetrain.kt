package com.teamcelestial.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs

object Drivetrain: SubsystemBase() {
    private val leftSlave = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)
    private val leftMaster = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)
    private val rightSlave = CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMaster = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)

    private val robotDrive =  DifferentialDrive(leftMaster, rightMaster)

    private lateinit var driveTrainPid: PIDController

    var pValue: NetworkValue<Double> = NetworkValue("P", NetworkValueType.kDouble, 0.0)
    var iValue: NetworkValue<Double> = NetworkValue("I", NetworkValueType.kDouble, 0.0)
    var dValue: NetworkValue<Double> = NetworkValue("D", NetworkValueType.kDouble, 0.0)

    init {
        rightMaster.inverted = true
        rightSlave.inverted = true
        leftSlave.follow(leftMaster)
        rightSlave.follow(rightMaster)
    }

    fun drive(x: Double, y: Double) {
        val steer = (x * 1.0 - (abs(y) * x * 0.6)) * 0.8
        robotDrive.arcadeDrive(y, steer)
    }

    private fun updateDriveTrainPIDController() {
        driveTrainPid = PIDController(pValue.value, iValue.value, dValue.value)
    }
}