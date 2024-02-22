package com.teamcelestial.commands

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.system.arm.ArmPresetData
import edu.wpi.first.wpilibj2.command.Command


class ArmControlCommand(
    private val supplier: () -> Double
) : Command() {
/*
    init {
        addRequirements(arm)
    }

    override fun initialize() {}

    override fun execute() {
        arm.setTargetTheta(150.0)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
    }*/
}
