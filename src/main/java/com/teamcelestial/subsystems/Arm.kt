package com.teamcelestial.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.teamcelestial.constant.BlindRotatorConstant
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.arm.ArmState
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Arm(
    private val armPresetData: ArmPresetData,
): SubsystemBase() {
    private var state: ArmState = ArmState(
        targetTheta = armPresetData.defaultTheta,
        theta = 0.0,
        output = 0.0,
        data = HashMap()
    )

    private var pValue: NetworkValue<Double> = NetworkValue("arm_P", NetworkValueType.kDouble, BlindRotatorConstant.kP)
    private var iValue: NetworkValue<Double> = NetworkValue("arm_I", NetworkValueType.kDouble, BlindRotatorConstant.kI)
    private var dValue: NetworkValue<Double> = NetworkValue("arm_D", NetworkValueType.kDouble, BlindRotatorConstant.kD)

    private val leftArm = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val rightArm = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)

    private var pidController: PIDController = PIDController(0.0, 0.0, 0.0)

    private val leftLimiter = SlewRateLimiter(0.3)
    private val rightLimiter = SlewRateLimiter(0.3)

    private lateinit var encoder: CANcoder //TODO: Add encoder

    init {
        val pValue: NetworkValue<Double> = NetworkValue("P", NetworkValueType.kDouble, 0.0)
        val iValue: NetworkValue<Double> = NetworkValue("I", NetworkValueType.kDouble, 0.0)
        val dValue: NetworkValue<Double> = NetworkValue("D", NetworkValueType.kDouble, 0.0)
        listOf(pValue, iValue, dValue).forEach {
            it.setListener { _ -> updatePIDController() }
        }
    }

    override fun periodic() {
        tick()
    }

    private fun setMotors(power: Double) {
        leftArm.set(leftLimiter.calculate(-power))
        rightArm.set(rightLimiter.calculate(power))
    }

    private fun tick() {
        updateTheta()
        updateOutput()
        updateMotors()
    }

    /**
     * Set the target angle for the arm.
     * @param theta The target angle in degrees.
     */
    fun setTargetTheta(theta: Double): Boolean {
        state = state.copy(targetTheta = theta)
        updateTarget()
        return true
    }

    private fun updateMotors() {
        setMotors(state.output)
    }

    private fun updateOutput() {
        state = state.copy(output = pidController.calculate(state.theta))
    }

    private fun updateTarget() {
        pidController.setpoint = state.targetTheta
    }

    private fun updateTheta() {
        state = state.copy(theta = transformEncoderOutputToDegrees(encoder.absolutePosition.value))
    }



    private fun transformEncoderOutputToDegrees(encoderOutput: Double): Double {
        return (encoderOutput * 360.0 - armPresetData.absZeroPointDegrees)
    }

    private fun updatePIDController() {
        pidController = PIDController(pValue.value, iValue.value, dValue.value)
        pidController.setpoint = state.targetTheta
    }
}