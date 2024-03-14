package com.teamcelestial.commands.feeder

import com.teamcelestial.subsystems.Feeder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class FeederControlCommand(
    private val feeder: Feeder,
    private val durationSeconds: Double,
    private val delay: Double = 0.0,
    private val powerConstant: Double = 1.0
) : Command() {

    private val timer = Timer()

    init {
        addRequirements(feeder)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        if(timer.get() < delay)
            feeder.setMotor(0.0)
        else
            feeder.setMotor(-0.2 * powerConstant)
    }

    override fun isFinished(): Boolean = timer.get() >= (durationSeconds + delay)

    override fun end(interrupted: Boolean) {
        feeder.setMotor(0.0)
        timer.reset()
    }
}
