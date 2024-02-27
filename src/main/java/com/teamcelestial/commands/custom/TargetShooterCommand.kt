package com.teamcelestial.commands.custom

import com.teamcelestial.commands.subsystem.ArmControlCommand
import com.teamcelestial.commands.subsystem.FeederControlCommand
import com.teamcelestial.commands.subsystem.RotatorControlCommand
import com.teamcelestial.commands.subsystem.ShooterControlCommand
import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Feeder
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class TargetShooterCommand(
    rotator: Rotator,
    arm: Arm,
    shooter: Shooter,
    feeder: Feeder,
    armTargetTheta: Double,
    rotatorTargetTheta: Double
) : SequentialCommandGroup() {

    private val delayedRotatorCommand = SequentialCommandGroup(
        WaitCommand(2.0),
        RotatorControlCommand(rotator, rotatorTargetTheta)
    )

    init {
        addCommands(
            ArmControlCommand(arm, armTargetTheta),
            RotatorControlCommand(rotator, rotatorTargetTheta),
            ParallelCommandGroup(
                FeederControlCommand(feeder, 3.0, 3.0),
                ShooterControlCommand(shooter, 6.0)
            )
        )
    }
}
