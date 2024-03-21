package com.teamcelestial.subsystems

import com.teamcelestial.Robot
import com.teamcelestial.system.assembly.ShooterAssemblyState
import com.teamcelestial.system.shooter.AbsoluteShooterTarget
import com.teamcelestial.system.shooter.ShooterTarget
import kotlin.math.absoluteValue

object ShooterAssembly {
    private lateinit var arm: Arm
    private lateinit var rotator: Rotator
    private lateinit var shooter: Shooter
    private lateinit var feeder: Feeder

    private var state = ShooterAssemblyState.idle
    private var shootingStart = 0L
    private var shooterSetpointMillis = 0L
    private var shooterSetpointLatch = false
    private const val shooterGracePeriodMillis = 250
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


    fun update() {
        arm.resetIntegrator()
        rotator.resetIntegrator()
        when (state) {
            ShooterAssemblyState.idle -> {
                feeder.setMotor(0.0)
                arm.setTargetTheta(90.0)
                rotator.setTargetTheta(180.0)
                shooter.stop() // TOOo: Add arm and rotator idle
                target = null
            }
            ShooterAssemblyState.wandering -> {
                arm.setTargetTheta(180.0)
                rotator.setTargetTheta(90.0)
                shooter.setTargetRPM(3000.0)
            }
            ShooterAssemblyState.arming -> {
                if(target == null) {
                    println("ShooterAssembly: No target available, aborting.")
                    state = ShooterAssemblyState.idle
                    return update()
                }
                if(targetPair == null) {
                    println("ShooterAssembly: INTERNAL ERROR, targetPair is null. Aborting.")
                    state = ShooterAssemblyState.idle
                    return update()
                }
                val thetaTarget = shooter.start(targetPair!!.first, targetPair!!.second, runMotors = true, rpmOverride = 3000.0).theta
                val armTarget = 128.0
                val link1Theta = armTarget - 90.0
                val link2Theta = thetaTarget + link1Theta
                val rotatorTarget = rotator.rotatorPreset.defaultTheta - link2Theta
                arm.setTargetTheta(armTarget)
                rotator.setTargetTheta(rotatorTarget)
            }
            ShooterAssemblyState.accelerating -> {
                shootBestAttempt()
            }
            ShooterAssemblyState.shooting -> {
                feeder.setMotor(-1.0)
            }
        }
    }

    private var lastRotatorThetaPublish = 0L

    fun tick() {
        when(state) {
            ShooterAssemblyState.arming -> {
                if(arm.atSetpoint && rotator.atSetpoint) {
                    state = ShooterAssemblyState.accelerating
                    update()
                }
            }
            ShooterAssemblyState.accelerating -> {
                if(shooter.atSetpoint()) {
                    if(shooterSetpointLatch && arm.atSetpoint && rotator.atSetpoint) {
                        if(System.currentTimeMillis() - shooterSetpointMillis >= shooterGracePeriodMillis) {
                            state = ShooterAssemblyState.shooting
                            shootingStart = System.currentTimeMillis()
                            update()
                        }
                    } else {
                        shooterSetpointMillis = System.currentTimeMillis()
                    }
                }
                shooterSetpointLatch = shooter.atSetpoint()
            }
            ShooterAssemblyState.shooting -> {
                if(System.currentTimeMillis() - shootingStart >= 1200) {
                    state = ShooterAssemblyState.idle
                    update()
                }
            }
            ShooterAssemblyState.wandering -> {
                val armThetaDiff = arm.getTheta() - 180
                if(armThetaDiff.absoluteValue <= 10) {
                    rotator.setTargetTheta(armThetaDiff + 90.0)
                }
            }
            else -> {}
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
        //println("Registering target: $shooterTarget")
        if (state != ShooterAssemblyState.wandering) return
        target = shooterTarget
        //state = ShooterAssemblyState.arming
    }

    fun shootBestAttempt() {
        val result = target!!.getTargetDistanceAndHeightPair(shooter)
        shooter.start(result.first, result.second, runMotors = true, thetaOverride = getShooterAbsTheta())
    }

    fun getState() = state
}