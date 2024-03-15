package com.teamcelestial

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.teamcelestial.commands.IntakeForwardCommand
import com.teamcelestial.commands.RotatorControlCommand
import com.teamcelestial.commands.ShooterControlCommand
import com.teamcelestial.commands.arm.ArmControlCommand
import com.teamcelestial.commands.custom.TargetShooterCommand
import com.teamcelestial.commands.custom.VisionBasedShootCommand
import com.teamcelestial.commands.drivetrain.RobotDriveCommand
import com.teamcelestial.commands.drivetrain.TurnToAngleCommand
import com.teamcelestial.commands.drivetrain.TurnToVisionTargetCommand
import com.teamcelestial.commands.feeder.FeederControlCommand
import com.teamcelestial.commands.feeder.FeederForwardCommand
import com.teamcelestial.commands.shooterAssembly.FinishWanderingCommand
import com.teamcelestial.commands.shooterAssembly.WanderingCommand
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.rotator.RotatorPresetData
import com.teamcelestial.vision.CameraOutput
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller

object RobotContainer {
    private val controller = PS4Controller(0)
    private val commandController = CommandPS4Controller(0)

    private val armPreset = ArmPresetData(
        defaultTheta = 135.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 292.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 115.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
    )

    private val desiredRotatorAngle = NetworkValue<Double>(
        "desiredRotatorAngle",
        NetworkValueType.kDouble,
        180.0
    )

    private val desiredArmAngle = NetworkValue<Double>(
        "desiredArmAngle",
        NetworkValueType.kDouble,
        180.0
    )

    private val desiredShooterRpm = NetworkValue(
        "desiredShooterRpm",
        NetworkValueType.kDouble,
        0.0
    )

    val arm = Arm(armPresetData = armPreset,)
    val rotator = Rotator(rotatorPreset = rotatorPreset)
    private val intake = Intake()
    private val feeder = Feeder()
    private val shooter = Shooter()

    val drivetrain = Drivetrain()

    init {
        initializeSubsystemDependencies()
        configureBindings()
    }

    private fun initializeSubsystemDependencies() {
        ShooterAssembly.initializeWithSubsystems(arm, rotator, shooter, feeder)

        NamedCommands.registerCommand(
            "takeNote",
            ParallelCommandGroup(
                FeederForwardCommand(feeder, -0.25),
                IntakeForwardCommand(intake, 0.7)
            )
        )

        NamedCommands.registerCommand(
            "closeDistanceShoot",
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    155.0,
                    52.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0),
                    ArmControlCommand(arm, 95.0, 15.0)
                )
            )
        )

        NamedCommands.registerCommand(
            "longDistanceShoot",
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    180.0,
                    50.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0),
                    ArmControlCommand(arm, 95.0)
                )
            )
        )

        arm.registerDisarmAvailabilityDependency(
            SubsystemCoherenceDependency(
                rotator.deploymentProvider
            )
        )
        rotator.registerDeploymentAvailabilityDependency(
            SubsystemCoherenceDependency(
                arm.availabilityProvider
            )
        )
    }

    private fun configureBindings() {
        drivetrain.defaultCommand = RobotDriveCommand(
            drivetrain,
            { controller.rightX },
            { controller.leftY },
            { if(controller.r1Button) 1.0 else 0.6 }
        )

        commandController.povDown().onTrue(
            ParallelCommandGroup(
                RotatorControlCommand(rotator, 180.0),
                ArmControlCommand(arm, 95.0)
            )
        )

        commandController.povUp().onTrue(
            ParallelCommandGroup(
                ArmControlCommand(arm, 180.0),
                RotatorControlCommand(rotator, 180.0)
            )
        )

        commandController.cross().onTrue(
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    180.0,
                    55.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0, 10.0),
                    ArmControlCommand(arm, 95.0, 15.0)
                )
            )
        )

        commandController.triangle().onTrue(
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    155.0,
                    55.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0, 10.0),
                    ArmControlCommand(arm, 95.0, 15.0)
                )
            )
        )

        commandController.square().onTrue(
            TurnToAngleCommand(drivetrain, 180.0)
        )

        commandController.povLeft().whileTrue(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ArmControlCommand(arm, 190.0),
                    RotatorControlCommand(rotator, 175.0)
                ),
                FeederForwardCommand(feeder, 0.6)
            )
        )

        listOf(desiredRotatorAngle, desiredArmAngle).forEach {
            it.setListener {
                commandController.povLeft().onTrue(
                    ParallelCommandGroup(
                        ArmControlCommand(arm, desiredArmAngle.value),
                        RotatorControlCommand(rotator, desiredRotatorAngle.value)
                    )
                )
            }
        }

        commandController.L1().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    IntakeForwardCommand(intake, 0.7),
                    FeederForwardCommand(feeder, -0.3)
                )
            )
        )

        commandController.povRight().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    IntakeForwardCommand(intake, -0.7),
                    FeederForwardCommand(feeder, 0.25)
                )
            )
        )

        commandController.L2()
            .onTrue(WanderingCommand(arm, rotator))
            .onFalse(FinishWanderingCommand(arm, rotator))
    }

    fun getAutonomousCommand(): Command {
        return PathPlannerAuto("Test")
    }
}