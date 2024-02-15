package com.teamcelestial.commands

import com.teamcelestial.subsystems.Feeder
import edu.wpi.first.wpilibj2.command.Command

class FeederControlCommand(
    private val supplier: () -> Double
): Command() {

    init {
        addRequirements(Feeder)
    }

    override fun initialize() {}

    override fun execute() {
        Feeder.setMotor(supplier())
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Feeder.setMotor(0.0)
    }
}