package com.teamcelestial

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.system.arm.ArmPresetData
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {

    private val armPreset = ArmPresetData(
        defaultTheta = 150.0, //TODO: The default theta, target angle when robot starts
        absZeroPointDegrees = 0.0 //TODO: The absolute zero point of the arm in degrees. Must be parallel to ground.
    )

    private val arm = Arm(
        armPresetData = armPreset
    )

    override fun robotInit() {
        RobotContainer
    }

    override fun robotPeriodic() {
        //CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        arm.resetIntegrator()
    }

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
