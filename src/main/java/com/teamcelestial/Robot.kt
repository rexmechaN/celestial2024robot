package com.teamcelestial

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.system.arm.ArmPresetData
import com.teamcelestial.system.coherence.SubsystemCoherenceDependency
import com.teamcelestial.system.rotator.RotatorPresetData
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {

    private val armPreset = ArmPresetData(
        defaultTheta = 135.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 0.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to ground.
    )

    private val arm = Arm(
        armPresetData = armPreset,
    )

    private val rotatorPreset = RotatorPresetData(
        defaultTheta = 180.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 102.0 //TODO: The absolute zero point of the arm in encoder units. Must be parallel to arm.
    )

    private val rotator = Rotator(
        rotatorPreset = rotatorPreset
    )

    private val joystick = Joystick(0)

    override fun robotInit() {
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
        rotator.setTargetTheta(90.0)
        arm.setTargetTheta(180.0)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    var joystickButtonLatch = false
    var mode = false

    override fun teleopInit() {
        arm.resetIntegrator()
        rotator.resetIntegrator()
    }

    override fun teleopPeriodic() {
        val button = joystick.getRawButton(7)
        val switchMode = button && !joystickButtonLatch
        if(switchMode) {
            mode = !mode
            if(mode) {
                arm.setTargetTheta(95.0)
            } else {
                arm.setTargetTheta(180.0)
            }
        }
        joystickButtonLatch = button
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
