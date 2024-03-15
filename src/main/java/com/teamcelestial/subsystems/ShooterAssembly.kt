package com.teamcelestial.subsystems

import com.teamcelestial.system.assembly.ShooterAssemblyState
import com.teamcelestial.system.shooter.ShooterTarget

object ShooterAssembly {
    private lateinit var arm: Arm
    private lateinit var rotator: Rotator
    private lateinit var shooter: Shooter
    private lateinit var feeder: Feeder

    private var state = ShooterAssemblyState.idle
    private var shootingStart = 0L
    private var shooterSetpointMillis = 0L
    private var shooterSetpointLatch = false
    private const val shooterGracePeriodMillis = 3000
    private var target: ShooterTarget? = null
        set(value) {
            field = value
            targetPair = value?.getTargetDistanceAndHeightPair(shooter)
        }
    private var targetPair: Pair<Double, Double>? = null

    fun initializeWithSubsystems(arm: Arm, rotator: Rotator, shooter: Shooter, feeder: Feeder) {
        this.arm = arm
        this.rotator = rotator
        this.shooter = shooter
        this.feeder = feeder
    }

    fun getShooterAbsTheta(): Double {
        val link1Theta = arm.getTheta() - 90.0
        val link2Theta = -(rotator.getTheta() - rotator.rotatorPreset.defaultTheta)
        return link2Theta - link1Theta
    }

    fun getLink1Theta(): Double {
        return arm.getTheta() - 90.0
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
                feeder.setMotor(0.0)
                shooter.stop() // TOOo: Add arm and rotator idle
                target = null
            }
            ShooterAssemblyState.wandering -> {
                arm.setTargetTheta(180.0)
                rotator.setTargetTheta(90.0)
                shooter.stop()
            }
            ShooterAssemblyState.arming -> {
                if(target == null) {
                    println("ShooterAssembly: No target available, aborting.")
                    state = ShooterAssemblyState.idle
                    update()
                }
                if(targetPair == null) {
                    println("ShooterAssembly: INTERNAL ERROR, targetPair is null. Aborting.")
                    state = ShooterAssemblyState.idle
                    update()
                }
                val thetaTarget = shooter.start(targetPair!!.first, targetPair!!.second, runMotors = true).theta
                val link1Theta = 130.0
                val link2Theta = thetaTarget + link1Theta
                arm.setTargetTheta(link1Theta)
                rotator.setTargetTheta(link2Theta)
            }
            ShooterAssemblyState.accelerating -> {
                shootBestAttempt()
            }
            ShooterAssemblyState.shooting -> {
                feeder.setMotor(-1.0)
            }
        }
    }

    fun tick() {
        when(state) {
            ShooterAssemblyState.arming -> {
                if(arm.atSetpoint && rotator.atSetpoint) {
                    state = ShooterAssemblyState.shooting
                    update()
                }
            }
            ShooterAssemblyState.accelerating -> {
                if(shooter.atSetpoint()) {
                    if(shooterSetpointLatch && System.currentTimeMillis() - shooterSetpointMillis >= shooterGracePeriodMillis){
                        state = ShooterAssemblyState.shooting
                        shootingStart = System.currentTimeMillis()
                        update()
                    } else {
                        shooterSetpointMillis = System.currentTimeMillis()
                    }
                }
                shooterSetpointLatch = shooter.atSetpoint()
            }
            ShooterAssemblyState.shooting -> {
                if(System.currentTimeMillis() - shootingStart >= 1500) {
                    state = ShooterAssemblyState.idle
                    update()
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    fun startWander() {
        state = ShooterAssemblyState.wandering
        update()
    }

    fun finishWander() {
        state = ShooterAssemblyState.arming
        update()
    }

    fun cancelWander() {
        state = ShooterAssemblyState.idle
        update()
    }

    fun aimForTarget(shooterTarget: ShooterTarget) {
        throw NotImplementedError("Use registerTarget instead.")
    }

    fun registerTarget(shooterTarget: ShooterTarget) {
        println("Registering target: $shooterTarget")
        if (state != ShooterAssemblyState.wandering) return
        target = shooterTarget
        //state = ShooterAssemblyState.arming
    }

    fun shootBestAttempt() {
        val result = target!!.getTargetDistanceAndHeightPair(shooter)
        shooter.start(result.first, result.second, runMotors = true, thetaOverride = getShooterAbsTheta())
    }
}