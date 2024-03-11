package com.teamcelestial

import com.teamcelestial.commands.subsystem.RobotDriveCommand
import com.teamcelestial.subsystems.Drivetrain
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

object SysIdContainer {
    private val controller = PS4Controller(0)
    private val commandController = CommandPS4Controller(0)

    private val drivetrain = Drivetrain()

    init {
        configureBindings()
    }

    private fun configureBindings() {
        drivetrain.defaultCommand = RobotDriveCommand(
            drivetrain,
            { controller.rightX },
            { controller.leftY },
            { if(controller.r1Button) 1.0 else 0.6 }
        )

        commandController.triangle().whileTrue(
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )

        commandController.cross().whileTrue(
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )

        commandController.square().whileTrue(
            drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )

        commandController.circle().whileTrue(
            drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
    }
}