package com.teamcelestial

import com.teamcelestial.subsystems.*
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {

    private lateinit var autonomousCommand: Command

    override fun robotInit() {
        RobotContainer
        autonomousCommand = RobotContainer.getAutonomousCommand()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        /*if(cameraOutput.bestTarget != null){
            val visionTarget = RelativeShooterTarget(cameraOutput.bestTarget!!.y, cameraOutput.bestTarget!!.z, 25.0)
            println(visionTarget.getTargetDistanceAndHeightPair(shooter))
        }*/
        SmartDashboard.putNumber("Shooter Abs Theta", ShooterAssembly.getShooterAbsTheta())
    }

    override fun autonomousInit() {
        RobotContainer.arm.resetIntegrator()
        RobotContainer.rotator.resetIntegrator()
        autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {}

    override fun autonomousExit() {
        autonomousCommand.cancel()
    }

    override fun teleopInit() {
        RobotContainer.arm.resetIntegrator()
        RobotContainer.rotator.resetIntegrator()
        //ShooterAssembly.wander()
    }

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
