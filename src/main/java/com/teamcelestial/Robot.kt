package com.teamcelestial

import com.teamcelestial.commands.arm
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {

    override fun robotInit() {
        RobotContainer
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
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
