package com.teamcelestial.commands.feeder

import com.teamcelestial.subsystems.Feeder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class FeederSetterCommand(
    private val feeder: Feeder,
    private val power: Double
) : Command() {
    init {
        addRequirements(feeder)
    }

    override fun initialize() {}

    override fun execute() {
        feeder.setMotor(power)
    }

    override fun isFinished(): Boolean = true

    override fun end(interrupted: Boolean) {}
}
