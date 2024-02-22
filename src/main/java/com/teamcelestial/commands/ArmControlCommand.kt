package com.teamcelestial.commands

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.system.arm.ArmPresetData
import edu.wpi.first.wpilibj2.command.Command

private val armPreset = ArmPresetData(
    defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
    absZeroPointDegrees = 0.0 //TODO: The absolute zero point of the arm in degrees. Must be parallel to ground.
)

val arm = Arm(
    armPresetData = armPreset
)

class ArmControlCommand(
    private val supplier: () -> Double
) : Command() {

    init {
        addRequirements(arm)
    }

    override fun initialize() {}

    override fun execute() {
        arm.setTargetTheta(150.0)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
    }
}
