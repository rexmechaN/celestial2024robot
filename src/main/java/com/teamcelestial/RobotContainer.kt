package com.teamcelestial

import com.teamcelestial.commands.*
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.subsystems.Drivetrain
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton

object RobotContainer {
    private val joystick = Joystick(0)

    init {
        Drivetrain.defaultCommand = DriveRobotCommand(
            { joystick.z * 0.3 },
            { joystick.y * 0.3 }
        )

        JoystickButton(joystick, 7).whileTrue(
            RepeatCommand( ArmControlCommand { 0.1 } )
        )

        JoystickButton(joystick, 8).whileTrue(
            RepeatCommand( ArmControlCommand { -0.1 } )
        )

        JoystickButton(joystick, 9).whileTrue(
            RepeatCommand( ShooterControlCommand { -0.2 } )
        )

        JoystickButton(joystick, 10).whileTrue(
            RepeatCommand( FeederControlCommand { -0.2 } )
        )

        JoystickButton(joystick, 11).whileTrue(
            RepeatCommand( RotatorControlCommand { -0.1 } )
        )

        JoystickButton(joystick, 12).whileTrue(
            RepeatCommand( RotatorControlCommand { 0.1 } )
        )
    }
}