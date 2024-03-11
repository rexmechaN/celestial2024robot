package com.teamcelestial

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.path.PathPlannerPath
import com.teamcelestial.commands.*
import com.teamcelestial.commands.custom.TargetShooterCommand
import com.teamcelestial.commands.subsystem.*
import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.rotator.RotatorPresetData
import com.teamcelestial.vision.CameraOutput
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.JoystickButton

object RobotContainer {
    private val controller = PS4Controller(0)
    private val commandController = CommandPS4Controller(0)

    private val armPreset = ArmPresetData(
        defaultTheta = 135.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 292.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 313.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
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
        ShooterAssembly.initializeWithSubsystems(arm, rotator, shooter)

        NamedCommands.registerCommand(
            "takeNote",
            FeederForwardCommand(feeder, -0.25)
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
                    60.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0),
                    ArmControlCommand(arm, 95.0)
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
                    RotatorControlCommand(rotator, 180.0),
                    ArmControlCommand(arm, 95.0)
                )
            )
        )

        commandController.circle().onTrue(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ArmControlCommand(arm, 158.0),
                    RotatorControlCommand(rotator, 60.0)
                ),
                ShooterControlCommand(shooter, 1.5, -500.0)
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
                    60.0
                ),
                ParallelCommandGroup(
                    RotatorControlCommand(rotator, 180.0),
                    ArmControlCommand(arm, 95.0)
                )
            )
        )

        commandController.square().onTrue(
            TurnToAngleCommand(drivetrain, 180.0)
        )

        commandController.povLeft().whileTrue(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ArmControlCommand(arm, 180.0),
                    RotatorControlCommand(rotator, 210.0)
                ),
                FeederForwardCommand(feeder, 0.8)
            )
        )

        commandController.L1().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    IntakeForwardCommand(intake, 0.7),
                    FeederForwardCommand(feeder, -0.25)
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
    }

    fun getAutonomousCommand(): Command {
        val path = PathPlannerPath.fromPathFile("Test2")
        return AutoBuilder.followPath(path)
    }
}