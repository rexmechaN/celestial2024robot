package com.teamcelestial.commands

import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj2.command.Command
class ShooterControlCommand(
    private val shooter: Shooter,
    private val rpm: Double
) : Command() {

    init {
        addRequirements(shooter)
    }

    override fun initialize() {
        shooter.setTargetRPM(rpm)
    }

    override fun execute() {
        //if(shooter.atSetpoint()) timer.start()
    }

    override fun isFinished(): Boolean = rpm == 0.0 || shooter.atSetpoint()

    override fun end(interrupted: Boolean) {
        //shooter.setMotor(0.0)
    }
}
