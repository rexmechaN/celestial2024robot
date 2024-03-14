package com.teamcelestial.commands

import com.teamcelestial.subsystems.Intake
import edu.wpi.first.wpilibj2.command.Command

class IntakeForwardCommand(
    private val intake: Intake,
    private val power: Double
) : Command() {

    init {
        addRequirements(intake)
    }

    override fun initialize() {}

    override fun execute() {
        intake.setMotor(power)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {intake.setMotor(0.0)}
}
