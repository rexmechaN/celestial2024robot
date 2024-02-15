package com.teamcelestial.commands

import com.teamcelestial.subsystems.Arm
import edu.wpi.first.wpilibj2.command.Command
class ArmControlCommand(
    private val supplier: () -> Double
) : Command() {

    init {
        addRequirements(Arm)
    }

    override fun initialize() {}

    override fun execute() {
        Arm.setMotors(supplier())
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Arm.setMotors(0.0)
    }
}
