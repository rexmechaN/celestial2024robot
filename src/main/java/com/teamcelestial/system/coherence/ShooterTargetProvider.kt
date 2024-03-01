package com.teamcelestial.system.coherence

data class ShooterTargetProvider (
    val provider: () -> Boolean = { true }
)