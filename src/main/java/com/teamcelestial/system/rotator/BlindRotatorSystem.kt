@file:Suppress("SameParameterValue")

package com.teamcelestial.system.rotator

import com.revrobotics.CANSparkMax
import com.teamcelestial.constant.BlindRotatorConstant
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.system.rotator.state.CalibrationData
import com.teamcelestial.system.rotator.state.PresetData
import com.teamcelestial.system.rotator.state.RotatorState
import com.teamcelestial.system.rotator.state.RotatorStatus
import edu.wpi.first.math.controller.PIDController
import kotlin.math.abs
import kotlin.math.min

class BlindRotatorSystem(
    presetData: PresetData,
    private val motors: Pair<CANSparkMax, CANSparkMax>
): IRotatorSystem {
    private var pValue: NetworkValue<Double> = NetworkValue("rotator_P", NetworkValueType.kDouble, BlindRotatorConstant.kP)
    private var iValue: NetworkValue<Double> = NetworkValue("rotator_I", NetworkValueType.kDouble, BlindRotatorConstant.kI)
    private var dValue: NetworkValue<Double> = NetworkValue("rotator_D", NetworkValueType.kDouble, BlindRotatorConstant.kD)

    private lateinit var pidController: PIDController

    private var state = RotatorState(
        status = RotatorStatus.idle,
        theta = 0.0,
        targetTheta = presetData.defaultTheta,
        output = 0.0,
        calibrationData = CalibrationData(
            endpoints = Pair(0.0, 0.0)
        ),
        presetData = presetData
    )
    override fun prime() {
        init()
        putData(BlindRotatorConstant.DataKey.PRIME_START_MILLIS, System.currentTimeMillis())
        setStatus(RotatorStatus.priming)
    }
    private fun init() {
        val pValue: NetworkValue<Double> = NetworkValue("P", NetworkValueType.kDouble, 0.0)
        val iValue: NetworkValue<Double> = NetworkValue("I", NetworkValueType.kDouble, 0.0)
        val dValue: NetworkValue<Double> = NetworkValue("D", NetworkValueType.kDouble, 0.0)
        listOf(pValue, iValue, dValue).forEach {
            it.setListener { _ -> updatePIDController() }
        }
    }
    override fun isReady(): Boolean = state.status == RotatorStatus.ready
    override fun setTargetTheta(theta: Double): Boolean = mutate(state.copy(targetTheta = theta)).status == RotatorStatus.ready
    override fun getTheta(): Double = state.theta
    override fun getThetaTarget(): Double = state.targetTheta
    override fun tick() {
        updateTheta()

        when (state.status) {
            RotatorStatus.idle -> {}
            RotatorStatus.priming -> {
                val primeStartMillis = getData(BlindRotatorConstant.DataKey.PRIME_START_MILLIS, 0L) as Long
                val duration = System.currentTimeMillis() - primeStartMillis
                if (duration < BlindRotatorConstant.ONE_SIDE_PRIME_PERIOD_MS) {
                    mutate(state.copy(output = BlindRotatorConstant.PRIME_OUTPUT, calibrationData = state.calibrationData.copy(endpoints = state.calibrationData.endpoints.copy(first = state.theta))))
                } else if (duration < BlindRotatorConstant.ONE_SIDE_PRIME_PERIOD_MS * 2) {
                    mutate(state.copy(output = -1.0 * BlindRotatorConstant.PRIME_OUTPUT, calibrationData = state.calibrationData.copy(endpoints = state.calibrationData.endpoints.copy(second = state.theta))))
                } else {
                    setStatus(RotatorStatus.ready)
                }
            }
            RotatorStatus.ready -> {
                mutate(state.copy(output = pidController.calculate(state.theta)))
            }
        }

        updateMotorOutput()
    }
    private fun updatePIDController() {
        pidController = PIDController(pValue.value, iValue.value, dValue.value)
        pidController.setpoint = state.targetTheta
    }
    private fun updateMotorOutput() {
        motors.first.set(state.output)
        motors.second.set(state.output)
    }
    private fun updateTheta() {
        val theta = map(state.calibrationData.endpoints, state.presetData.endpointAbsDegrees, state.theta)
        mutate(state.copy(theta = theta))
    }
    private fun map(sourceRange: Pair<Double, Double>, targetRange: Pair<Double, Double>, value: Double): Double {
        val sourceRangeSize = abs(sourceRange.second - sourceRange.first)
        val targetRangeSize = abs(targetRange.second - targetRange.first)
        if(sourceRangeSize == 0.0) return targetRange.first
        val valueScaled = abs((value - sourceRange.first)) / sourceRangeSize
        return min(targetRange.first, targetRange.second) + (valueScaled * targetRangeSize)
    }
    private fun mutate(rotatorState: RotatorState): RotatorState = rotatorState.also { state = it }
    private fun putData(key: String, value: Any): RotatorState = mutate(state.copy(data = state.data.plus(key to value)))
    private fun getData(key: String, defaultValue: Any? = null): Any? = state.data[key] ?: defaultValue
    private fun setStatus(status: RotatorStatus): RotatorState = mutate(state.copy(status = status))
}