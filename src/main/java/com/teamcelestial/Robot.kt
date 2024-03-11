package com.teamcelestial

import com.ctre.phoenix6.hardware.CANcoder
import com.teamcelestial.subsystems.*
import com.teamcelestial.system.shooter.RelativeShooterTarget
import com.teamcelestial.vision.CameraOutput
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {

    private lateinit var autonomousCommand: Command
    private val cameraOutput = CameraOutput("celestial")

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
        //SmartDashboard.putNumber("Shooter Abs Theta", ShooterAssembly.getShooterAbsTheta())
    }

    override fun autonomousInit() {
        RobotContainer.drivetrain.resetOdometry(
            Pose2d()
        )
        RobotContainer.drivetrain.resetSensors()
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
    }

    override fun teleopPeriodic() {
        if(cameraOutput.bestTarget != null) {
            println("X: ${cameraOutput.bestTarget!!.x}")
            println("Y: ${cameraOutput.bestTarget!!.y}")
            println("Z: ${cameraOutput.bestTarget!!.z}")
            //val target = RelativeShooterTarget(cameraOutput.bestTarget!!.y, cameraOutput.bestTarget!!.z, 0.0, 0.0)
            //ShooterAssembly.setShooterTarget(target)
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
