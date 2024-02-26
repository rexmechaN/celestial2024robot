package com.teamcelestial

import com.teamcelestial.commands.*
import com.teamcelestial.commands.custom.TargetShooterCommand
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.rotator.RotatorPresetData
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton

object RobotContainer {
    /*private val joystick = Joystick(0)

    private val armPreset = ArmPresetData(
        defaultTheta = 135.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 0.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    private val arm = Arm(
        armPresetData = armPreset,
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 102.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
    )

    private val rotator = Rotator(
        rotatorPreset = rotatorPreset
    )

    private val shooter = Shooter()
    private val feeder = Feeder()

    init {
        JoystickButton(joystick, 7).onTrue(
            TargetShooterCommand(
                rotator,
                arm,
                shooter,
                feeder,
                180.0,
                55.0
            )
        )
    }*/
}