package com.teamcelestial

import com.teamcelestial.commands.custom.TargetShooterCommand
import com.teamcelestial.commands.subsystem.*
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.rotator.RotatorPresetData
import com.teamcelestial.vision.CameraOutput
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller

object Robot : TimedRobot() {
    private val controller = PS4Controller(0)
    private val commandController = CommandPS4Controller(0)

    private val armPreset = ArmPresetData(
        defaultTheta = 135.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = -65.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    private val arm = Arm(
        armPresetData = armPreset,
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 265.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
    )

    private val rotator = Rotator(
        rotatorPreset = rotatorPreset
    )

    private val shooter = Shooter()
    private val feeder = Feeder()

    private val intake = Intake()

    private val cameraOutput = CameraOutput("celestial")

    override fun robotInit() {
        ShooterAssembly.initializeWithSubsystems(arm, rotator)

        RobotContainer
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

        Drivetrain.defaultCommand = RobotDriveCommand(
            { controller.rightX },
            { controller.leftY },
            { if(controller.r2Button) 1.0 else 0.6 }
        )

        commandController.povDown().onTrue(
            SequentialCommandGroup(
                RotatorControlCommand(rotator, 180.0),
                ArmControlCommand(arm, 95.0)
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
                RotatorControlCommand(rotator, 180.0),
                ArmControlCommand(arm, 95.0)
            )
        )

        commandController.L1().whileTrue(
            RepeatCommand(
                ParallelCommandGroup(
                    IntakeForwardCommand(intake, 0.6),
                    FeederForwardCommand(feeder, -0.2)
                )
            )
        )

        commandController.R1().whileTrue(
            SequentialCommandGroup(
                IntakeForwardCommand(intake, -0.6),
                FeederForwardCommand(feeder, 0.2)
            )
        )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        /*if(cameraOutput.bestTarget != null){
            val visionTarget = RelativeShooterTarget(cameraOutput.bestTarget!!.y, cameraOutput.bestTarget!!.z, 25.0)
            println(visionTarget.getTargetDistanceAndHeightPair(shooter))
        }*/

        //println(ShooterAssembly.getShooterAbsTheta())
    }

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        arm.resetIntegrator()
        rotator.resetIntegrator()
    }

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
