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
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.pow

class Arm(
    private val armPresetData: ArmPresetData,
): SubsystemBase() {
    private var state: ArmState = ArmState(
        targetTheta = armPresetData.defaultTheta,
        theta = 0.0,
        output = 0.0,
        data = HashMap()
    )

    private var pValue: NetworkValue<Double> = NetworkValue("arm_P", NetworkValueType.kDouble, 20.0)
    private var iValue: NetworkValue<Double> = NetworkValue("arm_I", NetworkValueType.kDouble, 20.0)
    private var dValue: NetworkValue<Double> = NetworkValue("arm_D", NetworkValueType.kDouble, 20.0)

    private val leftArm = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val rightArm = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)

    private var pidController: PIDController = PIDController(0.0, 0.0, 0.0)

    private val leftLimiter = SlewRateLimiter(0.3)
    private val rightLimiter = SlewRateLimiter(0.3)

    private val encoder = CANcoder(17)

    private var lastLog: Long = 0L

    init {
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
        if(System.currentTimeMillis() - lastLog > 1000) {
            lastLog = System.currentTimeMillis()
            println("====================================")
            println("Arm.PID: ${pidController.p}, ${pidController.i}, ${pidController.d}")
            println("Arm.Theta: ${state.theta}")
            println("Arm.ThetaTarget: ${state.targetTheta}")
            println("Arm.Output: ${state.output}")
        }
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
        pidController = PIDController(10.0.pow(-1.0 * pValue.value), 10.0.pow(-1.0 * iValue.value), 10.0.pow(-1.0 * dValue.value))
        pidController.setpoint = state.targetTheta
    }
}