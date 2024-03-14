package com.teamcelestial.commands.arm

import com.teamcelestial.subsystems.Arm
import edu.wpi.first.wpilibj2.command.Command


class ArmForwardCommand(
    private val arm: Arm,
    private val power: Double
) : Command() {

    init {
        addRequirements(arm)
    }

    override fun initialize() {}

    override fun execute() {
        arm.setMotors(power)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {arm.setMotors(0.0)}
}
