package com.teamcelestial

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.teamcelestial.commands.intake.IntakeForwardCommand
import com.teamcelestial.commands.RotatorControlCommand
import com.teamcelestial.commands.ShooterControlCommand
import com.teamcelestial.commands.arm.ArmControlCommand
import com.teamcelestial.commands.custom.TargetShooterCommand
import com.teamcelestial.commands.drivetrain.RobotDriveCommand
import com.teamcelestial.commands.drivetrain.TurnToAngleCommand
import com.teamcelestial.commands.drivetrain.TurnToVisionTargetCommand
import com.teamcelestial.commands.feeder.FeederForwardCommand
import com.teamcelestial.commands.feeder.FeederSetterCommand
import com.teamcelestial.commands.intake.IntakeSetterCommand
import com.teamcelestial.commands.shooterAssembly.CancelWanderingCommand
import com.teamcelestial.commands.shooterAssembly.FinishWanderingCommand
import com.teamcelestial.commands.shooterAssembly.WanderingCommand
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.rotator.RotatorPresetData
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller

object RobotContainer {
    private val controller = PS4Controller(0)
    private val commandController = CommandPS4Controller(0)

    private val armPreset = ArmPresetData(
        defaultTheta = 90.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 299.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 112.0 - 54.0 - 47.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
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

    private val angleP = NetworkValue(
        "angleP",
        NetworkValueType.kDouble,
        0.03
    )

    private val angleFeedforward = NetworkValue(
        "angleF",
        NetworkValueType.kDouble,
        0.1
    )

    val arm = Arm(armPresetData = armPreset)
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
                FeederSetterCommand(feeder, -0.3),
                IntakeSetterCommand(intake, 0.7)
            )
        )

        NamedCommands.registerCommand(
            "disableIntake",
            ParallelCommandGroup(
                FeederSetterCommand(feeder, 0.0),
                IntakeSetterCommand(intake, 0.0)
            )
        )

        NamedCommands.registerCommand(
            "MiddleShoot",
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    128.0,
                    103.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0, 45.0),
                    ArmControlCommand(arm, 90.0, 45.0)
                )
            )
        )

        NamedCommands.registerCommand(
            "CloseShoot",
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    155.0,
                    55.0,
                    4000.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0, 45.0),
                    ArmControlCommand(arm, 95.0, 45.0)
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
        shooter.registerDeploymentAvailabilityDependency(
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
            CancelWanderingCommand(arm, rotator)
        )

        commandController.povUp().onTrue(
            ParallelCommandGroup(
                ArmControlCommand(arm, 180.0),
                RotatorControlCommand(rotator, 180.0)
            )
        )

        commandController.cross().whileTrue(
            rotator.runOnce {
                rotator.setTargetTheta(rotator.getTheta() - 0.005)
            }
        )

        commandController.circle().whileTrue(
            rotator.runOnce {
                rotator.setTargetTheta(rotator.getTheta() + 0.005)
            }
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
                    ArmControlCommand(arm, 90.0, 15.0)
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
                FeederForwardCommand(feeder, 0.6),
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
                    FeederForwardCommand(feeder, 0.3),
                    ShooterControlCommand(shooter, -500.0)
                )
            )
        ).onFalse(ShooterControlCommand(shooter, 0.0))

        commandController.L2()
            .onTrue(WanderingCommand(arm, rotator))
            .onFalse(FinishWanderingCommand(arm, rotator))

        commandController.circle().whileTrue(
            TurnToVisionTargetCommand(
                drivetrain,
                { Robot.cameraOutput.bestTarget?.yaw ?: 0.0 },
                { controller.leftY },
                angleP.value,
                angleFeedforward.value
            )
        )

        listOf(angleP, angleFeedforward).forEach {
            it.setListener {
                commandController.circle().whileTrue(
                    TurnToVisionTargetCommand(
                        drivetrain,
                        { Robot.cameraOutput.bestTarget?.yaw ?: 0.0 },
                        { controller.leftY },
                        angleP.value,
                        angleFeedforward.value
                    )
                )
            }
        }

        /*commandController.circle()
            .onTrue(ShooterControlCommand(shooter, 4500.0))
            .onFalse(ShooterControlCommand(shooter, 0.0))*/
    }

    fun getAutonomousCommand(): Command {
        return PathPlannerAuto("3 Note")
    }
}