package com.teamcelestial.system.shooter

data class ShooterCalcResult (
    val rpm: Double,
    val theta: Double,
) {
    override fun toString(): String {
        return "<ShooterCalcResult RPM: $rpm, Theta: $theta>"
    }
}