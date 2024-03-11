package com.teamcelestial.subsystems

import com.teamcelestial.system.assembly.ShooterAssemblyState
import com.teamcelestial.system.shooter.ShooterTarget

object ShooterAssembly {
    private lateinit var arm: Arm
    private lateinit var rotator: Rotator
    private lateinit var shooter: Shooter
    private var state = ShooterAssemblyState.idle
    private var targetTheta: Double = 0.0

    fun initializeWithSubsystems(arm: Arm, rotator: Rotator, shooter: Shooter) {
        this.arm = arm
        this.rotator = rotator
        this.shooter = shooter
    }

    fun getShooterAbsTheta(): Double {
        val link1Theta = arm.getTheta() - 90.0
        val link2Theta = -(rotator.getTheta() - rotator.rotatorPreset.defaultTheta)
        return link2Theta - link1Theta
    }

    fun wander() {
        state = ShooterAssemblyState.wandering
        update()
    }

    fun cancel() {
        state = ShooterAssemblyState.wandering
        update()
    }


    fun update() {
        arm.resetIntegrator()
        rotator.resetIntegrator()
        when (state) {
            ShooterAssemblyState.idle -> {
                shooter.stop() // TOOo: Add arm and rotator idle
            }
            ShooterAssemblyState.wandering -> {
                arm.setTargetTheta(180.0)
                rotator.setTargetTheta(90.0)
                shooter.stop()
            }
            ShooterAssemblyState.arming -> {
                TODO()
            }
            ShooterAssemblyState.shooting -> {
                TODO()

            }
        }
    }

    fun aimForTarget(shooterTarget: ShooterTarget) {
        val result = shooterTarget.getTargetDistanceAndHeightPair(shooter)
        shooter.start(result.first, result.second)
        //state = ShooterAssemblyState.arming
    }
}