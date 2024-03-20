package com.teamcelestial.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.arm.ArmState
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow

class Arm(
    private val armPresetData: ArmPresetData,
    private var disarmAvailabilityDependency: SubsystemCoherenceDependency? = null,
    ): SubsystemBase() {
    private var state: ArmState = ArmState(
        targetTheta = armPresetData.defaultTheta,
        theta = 0.0,
        output = 0.0,
        data = HashMap()
    )

    val availabilityProvider = {
        state.theta > 120.0
    }

    private var pValue: NetworkValue<Double> = NetworkValue("arm_P", NetworkValueType.kDouble, 2.10)
    private var iValue: NetworkValue<Double> = NetworkValue("arm_I", NetworkValueType.kDouble, 3.8)
    private var dValue: NetworkValue<Double> = NetworkValue("arm_D", NetworkValueType.kDouble, 3.0)

    private val leftArm = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val rightArm = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)

    private var pidController: PIDController = PIDController(0.0,0.0,0.0)

    private val encoder = CANcoder(5)

    init {
        updatePIDController()
        listOf(pValue, iValue, dValue).forEach {
            it.setListener { _ -> updatePIDController() }
        }
    }

    val atSetpoint: Boolean
        get() = abs(state.targetTheta - state.theta) <= 6.0

    fun atSpecificSetpoint(tolerance: Double): Boolean = abs(state.targetTheta - state.theta) <= tolerance

    override fun periodic() {
        tick()
    }

    fun setMotors(power: Double) {
        leftArm.set(-power)
        rightArm.set(power)
    }

    private fun tick() {
        updateTheta()
        updateOutput()
        updateMotors()

        SmartDashboard.putNumber("Arm PID-P:", pidController.p)
        SmartDashboard.putNumber("Arm PID-I:", pidController.i)
        SmartDashboard.putNumber("Arm PID-D:", pidController.d)
        SmartDashboard.putNumber("Arm Theta", state.theta)
        SmartDashboard.putNumber("Arm Target Theta", state.targetTheta)
        SmartDashboard.putNumber("Arm Output", state.output)
    }

    /**
     * Set the target angle for the arm.
     * @param theta The target angle in degrees.
     */
    fun setTargetTheta(theta: Double): Boolean {
        state = state.copy(targetTheta = theta)
        updateTarget()
        resetIntegrator()
        return true
    }

    /**
     * Reset PID controller integrator
     */
    fun resetIntegrator() {
        pidController.reset()
    }

    private fun updateMotors() {
        setMotors(state.output)
    }

    private fun updateOutput() {
        state = if(state.theta > 200)
            state.copy(output = 0.0)
        else
            state.copy(output = pidController.calculate(state.theta).let {
                if(availabilityProvider()) it else disarmAvailabilityDependency?.isReady()?.run {
                    if(this) it else max(0.0, it)
                } ?: it
            })
    }

    private fun updateTarget() {
        pidController.reset()
        pidController.setpoint = state.targetTheta
    }

    private fun updateTheta() {
        state = state.copy(theta = transformEncoderOutputToDegrees(encoder.absolutePosition.value))
    }


    private fun transformEncoderOutputToDegrees(encoderOutput: Double): Double {
        return (encoderOutput * 360.0 - armPresetData.absZeroPointDegrees).let {
            if(it < 0) it + 360.0 else it
        }
    }

    private fun updatePIDController() {
        pidController = PIDController(10.0.pow(-1.0 * pValue.value), 10.0.pow(-1.0 * iValue.value), 10.0.pow(-1.0 * dValue.value))
        pidController.setpoint = state.targetTheta
    }

    fun registerDisarmAvailabilityDependency(dependency: SubsystemCoherenceDependency) {
        disarmAvailabilityDependency = dependency
    }

    fun getTheta(): Double {
        return state.theta
    }
}