package com.teamcelestial.system.rotator

interface IRotatorSystem {
    fun prime()

    fun isReady(): Boolean

    fun setTargetTheta(theta: Double): Boolean

    fun getTheta(): Double

    fun getThetaTarget(): Double

    fun tick()
}