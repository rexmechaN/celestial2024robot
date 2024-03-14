package com.teamcelestial.commands

import com.teamcelestial.subsystems.Feeder
import com.teamcelestial.subsystems.Rotator
import edu.wpi.first.wpilibj2.command.Command

class RotatorControlCommand(
    private val rotator: Rotator,
    private val targetTheta: Double,
    private val tolerance: Double? = null
): Command() {

    var isSetted = false

    init {
        addRequirements(rotator)
    }

    override fun initialize() {}

    override fun execute() {
        if(!isSetted) {
            rotator.setTargetTheta(targetTheta)
            isSetted = true
        }
    }

    override fun isFinished(): Boolean =
        if(tolerance != null) rotator.atSpecificSetpoint(tolerance) else rotator.atSetpoint

    override fun end(interrupted: Boolean) { isSetted = false }
}