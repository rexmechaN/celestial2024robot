package com.teamcelestial

import com.teamcelestial.commands.custom.TargetShooterCommand
import com.teamcelestial.commands.subsystem.ArmControlCommand
import com.teamcelestial.commands.subsystem.ArmForwardCommand
import com.teamcelestial.commands.subsystem.RobotDriveCommand
import com.teamcelestial.commands.subsystem.RotatorControlCommand
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.rotator.RotatorPresetData
import com.teamcelestial.system.shooter.RelativeShooterTarget
import com.teamcelestial.vision.CameraOutput
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.photonvision.PhotonCamera

object Robot : TimedRobot() {

    private val joystick = Joystick(0)

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

        Drivetrain.defaultCommand = RobotDriveCommand({ joystick.x }, { joystick.y })

        JoystickButton(joystick, 7).onTrue(
            SequentialCommandGroup(
                TargetShooterCommand(
                    rotator,
                    arm,
                    shooter,
                    feeder,
                    180.0,
                    50.0
                ),
                RotatorControlCommand(rotator, 180.0),
                ArmControlCommand(arm, 120.0)
            )
        )

        JoystickButton(joystick, 9).onTrue(
            SequentialCommandGroup(
                RotatorControlCommand(rotator, 180.0),
                ArmControlCommand(arm, 180.0)
            )
        )

        JoystickButton(joystick, 10).onTrue(
            SequentialCommandGroup(
                RotatorControlCommand(rotator, 180.0),
                ArmControlCommand(arm, 120.0)
            )
        )
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        if(cameraOutput.bestTarget != null){
            val visionTarget = RelativeShooterTarget(cameraOutput.bestTarget!!.y, cameraOutput.bestTarget!!.z, 25.0)
            println(visionTarget.getTargetDistanceAndHeightPair(shooter))
        }
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
