package com.teamcelestial.commands.intake

import com.teamcelestial.subsystems.Feeder
import com.teamcelestial.subsystems.Intake
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class IntakeSetterCommand(
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

    override fun isFinished(): Boolean = true

    override fun end(interrupted: Boolean) {}
}
