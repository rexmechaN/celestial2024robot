package com.teamcelestial.subsystems

import com.teamcelestial.system.shooter.ShooterTarget

object ShooterAssembly {
    private lateinit var arm: Arm
    private lateinit var rotator: Rotator
    private lateinit var shooter: Shooter

    fun initializeWithSubsystems(arm: Arm, rotator: Rotator, shooter: Shooter) {
        this.arm = arm
        this.rotator = rotator
        this.shooter = shooter
    }

    fun getShooterAbsTheta(): Double {
        val link1Theta = 270.0 - arm.getTheta()
        val link2Theta = -(rotator.getTheta() - rotator.rotatorPreset.defaultTheta)
        return link1Theta + link2Theta
    }

    fun setShooterTarget(shooterTarget: ShooterTarget) {
        val result = shooterTarget.getTargetDistanceAndHeightPair(shooter)
        shooter.start(result.first, result.second)
    }
}