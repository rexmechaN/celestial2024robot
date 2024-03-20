package com.teamcelestial.commands.shooterAssembly

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.subsystems.ShooterAssembly
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs

class FinishWanderingCommand(
    private val arm: Arm,
    private val rotator: Rotator
): Command() {

    private var done = false

    init {
        addRequirements(arm, rotator)
    }

    override fun initialize() {}
    override fun execute() {
        println("FinishWanderingCall")
        if(!done && abs(arm.getTheta() - 180.0) <= 6.0 && abs(rotator.getTheta() - 90.0) <= 6.0) {
            println("FinishWanderingExecute")
            ShooterAssembly.finishWander()
            done = true
        } else {
            println("FinishWanderingAbort! done: $done, thetaDelta: ${abs(arm.getTheta() - 180.0)}, rotatorDelta: abs(rotator.getTheta() - 90.0)")
        }
    }
    override fun isFinished(): Boolean = arm.atSetpoint && rotator.atSetpoint
}