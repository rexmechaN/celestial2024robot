package com.teamcelestial

import com.teamcelestial.commands.DriveRobotCommand
import com.teamcelestial.subsystems.Drivetrain

object RobotContainer {
    init {
        Drivetrain.defaultCommand = DriveRobotCommand()
    }
}