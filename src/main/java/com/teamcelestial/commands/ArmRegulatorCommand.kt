package com.teamcelestial.commands

import com.teamcelestial.subsystems.Arm
import edu.wpi.first.wpilibj2.command.Command
class ArmRegulatorCommand : Command() {
    private val arm = Arm

    init {
        addRequirements(arm)
    }

    override fun initialize() {}

    override fun execute() {}

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {}
}
