package com.teamcelestial.system.rotator

data class RotatorState(
    val theta: Double,
    val targetTheta: Double,
    val output: Double,
    val data: Map<String, Any> = HashMap(),
    )