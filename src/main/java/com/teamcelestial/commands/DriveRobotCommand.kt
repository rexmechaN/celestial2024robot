package com.teamcelestial.commands

import edu.wpi.first.wpilibj2.command.Command
import com.teamcelestial.subsystems.Drivetrain
import com.teamcelestial.util.JoystickObject

class DriveRobotCommand : Command() {
    private val drivetrain = Drivetrain

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {}

    override fun execute() {
        val x = JoystickObject.singleton.z
        val y = JoystickObject.singleton.y
        drivetrain.drive(x, y)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {}
}
