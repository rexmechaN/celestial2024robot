package com.teamcelestial.system.coherence

data class SubsystemCoherenceDependency (
    val test: () -> Boolean = { true }
) {
    fun isReady(): Boolean {
        return test()
    }
}