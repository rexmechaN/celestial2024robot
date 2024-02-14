package com.teamcelestial.util

abstract class CelestialSubsystem {
    companion object {
        val subsystems: MutableList<CelestialSubsystem> = mutableListOf()
    }

    init {
        subsystems.add(this)
    }

    abstract fun tick()
}