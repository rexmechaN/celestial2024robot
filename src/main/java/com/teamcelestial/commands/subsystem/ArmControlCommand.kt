package com.teamcelestial.commands.subsystem

import com.teamcelestial.subsystems.Arm
import edu.wpi.first.wpilibj2.command.Command


class ArmControlCommand(
    private val arm: Arm,
    private val targetTheta: Double
) : Command() {

    var isSetted = false

    init {
        addRequirements(arm)
    }

    override fun initialize() {}

    override fun execute() {
        if(!isSetted) {
            arm.setTargetTheta(targetTheta)
            isSetted = true
        }
    }

    override fun isFinished(): Boolean = arm.atSetpoint

    override fun end(interrupted: Boolean) { isSetted = false }
}
