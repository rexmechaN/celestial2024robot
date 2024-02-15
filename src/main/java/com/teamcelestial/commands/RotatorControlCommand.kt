package com.teamcelestial.commands

import com.teamcelestial.subsystems.Feeder
import com.teamcelestial.subsystems.Rotator
import edu.wpi.first.wpilibj2.command.Command

class RotatorControlCommand(
    private val supplier: () -> Double
): Command() {

    init {
        addRequirements(Rotator)
    }

    override fun initialize() {}

    override fun execute() {
        Rotator.setMotors(supplier())
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Rotator.setMotors(0.0)
    }
}