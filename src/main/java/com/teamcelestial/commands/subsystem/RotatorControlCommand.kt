package com.teamcelestial.commands.subsystem

import com.teamcelestial.subsystems.Feeder
import com.teamcelestial.subsystems.Rotator
import edu.wpi.first.wpilibj2.command.Command

class RotatorControlCommand(
    private val rotator: Rotator,
    private val targetTheta: Double
): Command() {

    init {
        addRequirements(rotator)
    }

    override fun initialize() {}

    override fun execute() { rotator.setTargetTheta(targetTheta) }

    override fun isFinished(): Boolean = rotator.atSetpoint

    override fun end(interrupted: Boolean) {}
}