package com.teamcelestial.commands.subsystem

import com.teamcelestial.network.NetworkValue
import com.teamcelestial.network.NetworkValueType
import com.teamcelestial.subsystems.Drivetrain
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

class TurnToAngleCommand(
    private val drivetrain: Drivetrain,
    private val angle: Double,
): Command() {
    private val angularPid = PIDController(0.007, 0.0, 0.0)
    private var targetAngle: Double = 0.0

    private val timer = Timer()

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
        timer.start()
        targetAngle = drivetrain.getDegrees() + angle
        SmartDashboard.putNumber("Target Angle", targetAngle)
    }

    override fun execute() {
        val x = angularPid.calculate(drivetrain.getDegrees(), targetAngle)
        val output = min(abs(x), 0.8) * -x.sign
        SmartDashboard.putNumber("Target Output", output)
        drivetrain.drive(output, 0.0)
    }

    override fun isFinished(): Boolean = abs(drivetrain.getDegrees() - targetAngle) <= 8 || timer.get() > 5

    override fun end(interrupted: Boolean) {
        timer.reset()
    }
}