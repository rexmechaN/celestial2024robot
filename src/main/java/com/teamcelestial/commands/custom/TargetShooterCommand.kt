package com.teamcelestial.commands.custom

import com.teamcelestial.commands.RotatorControlCommand
import com.teamcelestial.commands.ShooterControlCommand
import com.teamcelestial.commands.arm.ArmControlCommand
import com.teamcelestial.commands.feeder.FeederControlCommand
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
    rotatorTargetTheta: Double,
    targetRpm: Double = 4500.0
) : SequentialCommandGroup() {

    init {
        addCommands(
            ParallelCommandGroup(
                ArmControlCommand(arm, armTargetTheta),
                RotatorControlCommand(rotator, rotatorTargetTheta)
            ),
            ShooterControlCommand(shooter, targetRpm),
            FeederControlCommand(feeder, 2.0, 0.0, -1.0),
            ShooterControlCommand(shooter, 0.0)
        )
    }
}