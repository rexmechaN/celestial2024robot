package com.teamcelestial.commands.subsystem

import com.teamcelestial.subsystems.Arm
import edu.wpi.first.wpilibj2.command.Command


class ArmControlCommand(
    private val arm: Arm,
    private val targetTheta: Double
) : Command() {

    init {
        addRequirements(arm)
    }

    override fun initialize() {}

    override fun execute() { arm.setTargetTheta(targetTheta) }

    override fun isFinished(): Boolean = arm.atSetpoint

    override fun end(interrupted: Boolean) {}
}
