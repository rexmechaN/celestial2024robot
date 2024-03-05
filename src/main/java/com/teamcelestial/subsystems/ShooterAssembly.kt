package com.teamcelestial.subsystems

object ShooterAssembly {
    private lateinit var arm: Arm
    private lateinit var rotator: Rotator

    fun initializeWithSubsystems(arm: Arm, rotator: Rotator) {
        this.arm = arm
        this.rotator = rotator
    }

    fun getShooterAbsTheta(): Double {
        val link1Theta = 270.0 - arm.getTheta()
        val link2Theta = rotator.getTheta() - rotator.rotatorPreset.defaultTheta
        return link1Theta + link2Theta
    }
}