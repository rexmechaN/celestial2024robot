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

    /*private val armPreset = ArmPresetData(
        defaultTheta = 135.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = -65.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    val arm = Arm(
        armPresetData = armPreset,
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 265.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
    )

    val rotator = Rotator(
        rotatorPreset = rotatorPreset
    )

    private val shooter = Shooter()
    private val feeder = Feeder()

    private val intake = Intake()

    private val cameraOutput = CameraOutput("celestial")

    private val desiredRotatorAngle = NetworkValue<Double>(
        "desiredRotatorAngle",
        NetworkValueType.kDouble,
        180.0
    )

    private val desiredArmAngle = NetworkValue<Double>(
        "desiredArmAngle",
        NetworkValueType.kDouble,
        180.0
    )*/

    init {
        initializeSubsystemDependencies()
        configureBindings()
    }

    private fun initializeSubsystemDependencies() {
        /*ShooterAssembly.initializeWithSubsystems(arm, rotator, shooter)

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
        )*/
    }

    private fun configureBindings() {
        Drivetrain.defaultCommand = RobotDriveCommand(
            { controller.rightX },
            { controller.leftY },
            { if(controller.r1Button) 1.0 else 0.6 }
        )

        /*commandController.povDown().onTrue(
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

        commandController.L1().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    IntakeForwardCommand(intake, 0.7),
                    FeederForwardCommand(feeder, -0.2)
                )
            )
        )

        commandController.povRight().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    IntakeForwardCommand(intake, -0.7),
                    FeederForwardCommand(feeder, 0.2)
                )
            )
        )

        desiredArmAngle.setListener {
            commandController.povLeft().onTrue(
                ParallelCommandGroup(
                    ArmControlCommand(arm, it),
                    RotatorControlCommand(rotator, desiredRotatorAngle.value)
                )
            )
        }

        desiredRotatorAngle.setListener {
            commandController.povLeft().onTrue(
                ParallelCommandGroup(
                    ArmControlCommand(arm, desiredArmAngle.value),
                    RotatorControlCommand(rotator, it)
                )
            )
        }*/
    }

    fun getAutonomousCommand(): Command {
        val path = PathPlannerPath.fromPathFile("Test")
        return AutoBuilder.followPath(path)
    }
}