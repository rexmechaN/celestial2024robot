package com.teamcelestial.commands

import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
class ShooterControlCommand(
    private val shooter: Shooter,
    private val rpm: Double,
    //private val maxDuration: Double = Integer.MAX_VALUE.toDouble()
) : Command() {

    private val timer = Timer()

    init {
        addRequirements(shooter)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
        shooter.setTargetRPM(rpm)
    }

    override fun execute() {
        //if(shooter.atSetpoint()) timer.start()
    }

    override fun isFinished(): Boolean = rpm == 0.0 || shooter.atSetpoint() // || (timer >= maxDuration)

    override fun end(interrupted: Boolean) {
        timer.stop()
        timer.reset()
        //shooter.setMotor(0.0)
    }
}
