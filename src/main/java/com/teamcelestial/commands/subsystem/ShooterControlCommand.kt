package com.teamcelestial.commands.subsystem

import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
class ShooterControlCommand(
    private val shooter: Shooter,
    private val durationSeconds: Double,
    private val delay: Double = 0.0
) : Command() {

    private val timer = Timer()

    init {
        addRequirements(shooter)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        if(timer.get() < delay)
            shooter.setMotor(0.0)
        else
            shooter.setMotor(4000.0)
    }

    override fun isFinished(): Boolean = timer.get() >= (durationSeconds + delay)

    override fun end(interrupted: Boolean) {
        shooter.setMotor(0.0)
        timer.reset()
    }
}
