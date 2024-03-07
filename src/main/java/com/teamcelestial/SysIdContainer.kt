package com.teamcelestial

import com.teamcelestial.commands.subsystem.RobotDriveCommand
import com.teamcelestial.subsystems.Drivetrain
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

object SysIdContainer {
    private val controller = PS4Controller(0)
    private val commandController = CommandPS4Controller(0)

    init {
        configureBindings()
    }

    private fun configureBindings() {
        Drivetrain.defaultCommand = RobotDriveCommand(
            { controller.rightX },
            { controller.leftY },
            { if(controller.r1Button) 1.0 else 0.6 }
        )

        commandController.triangle().whileTrue(
            Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )

        commandController.cross().whileTrue(
            Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )

        commandController.square().whileTrue(
            Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )

        commandController.circle().whileTrue(
            Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
    }
}