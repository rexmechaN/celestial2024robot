package com.teamcelestial.commands

import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj2.command.Command
class ShooterControlCommand(
    private val supplier: () -> Double
) : Command() {

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {}

    override fun execute() {
        Shooter.setMotors(supplier())
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Shooter.setMotors(0.0)
    }
}
