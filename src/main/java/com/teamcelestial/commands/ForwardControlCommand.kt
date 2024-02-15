package com.teamcelestial.commands

import com.teamcelestial.util.CelestialSubsystem
import edu.wpi.first.wpilibj2.command.Command

class ForwardControlCommand(
    private val supplier: () -> Double,
    private val subsystem: CelestialSubsystem
): Command() {

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {}

    override fun execute() {
        subsystem.setPower(supplier())
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        subsystem.setPower(0.0)
    }
}