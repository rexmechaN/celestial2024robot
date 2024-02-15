package com.teamcelestial.system.rotator.state

data class RotatorState(
    val status: RotatorStatus,
    val theta: Double,
    val targetTheta: Double,
    val output: Double,
    val calibrationData: CalibrationData,
    val presetData: PresetData,
    val data: Map<String, Any> = HashMap(),
    )

enum class RotatorStatus {
    idle,
    priming,
    ready,
}