package com.teamcelestial

import com.teamcelestial.subsystems.*
import com.teamcelestial.system.shooter.AbsoluteShooterTarget
import com.teamcelestial.vision.CameraOutput
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

    private val cameraOutput = CameraOutput("celestial")

    override fun teleopPeriodic() {
        ShooterAssembly.tick()
        if(cameraOutput.bestTarget != null) {
            val x = cameraOutput.bestTarget?.bestCameraToTarget?.x
            val y = cameraOutput.bestTarget?.bestCameraToTarget?.y
            val z = cameraOutput.bestTarget?.bestCameraToTarget?.z
            if(x != null && y != null && z != null) {
                val target = AbsoluteShooterTarget(x, 2.10
                )
                ShooterAssembly.registerTarget(target)
            }
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
