package com.teamcelestial.commands.drivetrain

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

class TurnToVisionTargetCommand(
    private val drivetrain: Drivetrain,
    private val targetAngleSupplier: () -> Double,
    private val forwardSupplier: () -> Double = {0.0},
    private val p: Double,
    private val feedforward: Double
): Command() {
    private val angularPid = PIDController(p, 0.000000000, 0.0)

    private val timer = Timer()

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
    }

    override fun execute() {
        //println(targetAngleSupplier())
        val x = angularPid.calculate(targetAngleSupplier(), 0.0) + feedforward
        val output = min(abs(x), 0.7) * -x.sign
        drivetrain.drive(output, forwardSupplier())
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {}
}