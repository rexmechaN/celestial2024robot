package com.teamcelestial.commands

import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
class ShooterControlCommand(
    private val shooter: Shooter,
    private val rpm: Double,
    private val durationSeconds: Double? = null,
) : Command() {

    private val timer = Timer()

    init {
        addRequirements(shooter)
    }

    override fun initialize() {
        timer.reset()
    }

    override fun execute() {
        shooter.setMotor(rpm)
        if(shooter.atSetpoint()) timer.start()
    }

    override fun isFinished(): Boolean = durationSeconds != null && timer.hasElapsed(durationSeconds)

    override fun end(interrupted: Boolean) {
        shooter.setMotor(0.0)
    }
}
