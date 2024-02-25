package com.teamcelestial.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.system.arm.ArmState
import com.teamcelestial.system.rotator.RotatorPresetData
import com.teamcelestial.system.rotator.RotatorState
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.min
import kotlin.math.pow

class Rotator(
    private val rotatorPreset: RotatorPresetData,
): SubsystemBase() {
    private val leftMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val rightMotor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

    private var state: RotatorState = RotatorState(
        targetTheta = rotatorPreset.defaultTheta,
        theta = 0.0,
        output = 0.0,
        data = HashMap()
    )

    private var pValue: NetworkValue<Double> = NetworkValue("rot_P", NetworkValueType.kDouble, 20.0)
    private var iValue: NetworkValue<Double> = NetworkValue("rot_I", NetworkValueType.kDouble, 20.0)
    private var dValue: NetworkValue<Double> = NetworkValue("rot_D", NetworkValueType.kDouble, 20.0)

    private var pidController: PIDController = PIDController(0.0, 0.0, 0.0)

    private val encoder = CANcoder(19) // TODO: Change CANcoder ID

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
        leftMotor.set(-power)
        rightMotor.set(power)
    }

    private fun tick() {
        val c1 = System.currentTimeMillis()
        updateTheta()
        val c2 = System.currentTimeMillis()
        updateOutput()
        val c3 = System.currentTimeMillis()
        updateMotors()
        val c4 = System.currentTimeMillis()
        if(System.currentTimeMillis() - lastLog > 1000) {
            lastLog = System.currentTimeMillis()
            println("====================================")
            println("Rotator.PID: ${pidController.p}, ${pidController.i}, ${pidController.d}")
            println("Rotator.Theta: ${state.theta}")
            println("Rotator.ThetaTarget: ${state.targetTheta}")
            println("Rotator.Output: ${state.output}")
        }
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
        state = if(state.theta < 70)
            state.copy(output = min(0.0, pidController.calculate(state.theta)))
        else
            state.copy(output = pidController.calculate(state.theta))
    }

    private fun updateTarget() {
        pidController.reset()
        pidController.setpoint = state.targetTheta
    }

    private fun updateTheta() {
        state = state.copy(theta = transformEncoderOutputToDegrees(encoder.absolutePosition.value))
    }


    private fun transformEncoderOutputToDegrees(encoderOutput: Double): Double {
        return (encoderOutput * 360.0 - rotatorPreset.absZeroPointDegrees).let {
            if(it < 0) it + 360.0 else it
        }
    }

    private fun updatePIDController() {
        pidController = PIDController(10.0.pow(-1.0 * pValue.value), 10.0.pow(-1.0 * iValue.value), 10.0.pow(-1.0 * dValue.value))
        pidController.setpoint = state.targetTheta
    }
}