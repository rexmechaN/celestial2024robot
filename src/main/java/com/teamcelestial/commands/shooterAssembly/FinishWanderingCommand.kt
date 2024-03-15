package com.teamcelestial.commands.shooterAssembly

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.subsystems.ShooterAssembly
import edu.wpi.first.wpilibj2.command.Command

class FinishWanderingCommand(
    private val arm: Arm,
    private val rotator: Rotator
): Command() {
    init {
        addRequirements(arm, rotator)
    }

    override fun initialize() {
        ShooterAssembly.finishWander()
    }
    override fun execute() {}
    override fun isFinished(): Boolean = arm.atSetpoint && rotator.atSetpoint
}