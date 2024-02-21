package com.teamcelestial.system.arm

data class ArmState(
    val theta: Double,
    val targetTheta: Double,
    val output: Double,
    val data: Map<String, Any> = HashMap(),
    )