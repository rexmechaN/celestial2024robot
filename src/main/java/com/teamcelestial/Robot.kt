package com.teamcelestial

import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Drivetrain
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.subsystems.Shooter
import com.teamcelestial.util.CelestialSubsystem
import edu.wpi.first.wpilibj.TimedRobot

object Robot : TimedRobot() {
    private fun initializeSubsystems() {
        Shooter; Arm; Rotator; Drivetrain
    }

    override fun robotInit() {
        initializeSubsystems()
    }

    override fun robotPeriodic() {}

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    override fun teleopPeriodic() {
        CelestialSubsystem.subsystems.forEach {
            it.tick()
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
