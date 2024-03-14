package com.teamcelestial.commands.custom

import com.teamcelestial.commands.arm.ArmControlCommand
import com.teamcelestial.commands.feeder.FeederControlCommand
import com.teamcelestial.commands.RotatorControlCommand
import com.teamcelestial.commands.ShooterControlCommand
import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Feeder
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.subsystems.Shooter
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class TargetShooterCommand(
    rotator: Rotator,
    arm: Arm,
    shooter: Shooter,
    feeder: Feeder,
    armTargetTheta: Double,
    rotatorTargetTheta: Double
) : SequentialCommandGroup() {

    init {
        addCommands(
            ParallelCommandGroup(
                ArmControlCommand(arm, armTargetTheta),
                RotatorControlCommand(rotator, rotatorTargetTheta)
            ),
            ParallelCommandGroup(
                FeederControlCommand(feeder, 0.2, 0.0, -1.0),
                ShooterControlCommand(shooter, 4000.0,2.0)
            ),
            FeederControlCommand(feeder, 2.0, 0.4),
        )
    }
}
