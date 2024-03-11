package com.teamcelestial.commands.subsystem

import edu.wpi.first.wpilibj2.command.Command
import com.teamcelestial.subsystems.Drivetrain

class RobotDriveCommand(
    private val drivetrain: Drivetrain,
    private val xSupplier: () -> Double,
    private val ySupplier: () -> Double,
    private val limiterSupplier: () -> Double = { 1.0 }
) : Command() {

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {}

    override fun execute() {
        val x = xSupplier() * limiterSupplier()
        val y = ySupplier() * limiterSupplier()
        drivetrain.drive(x, y)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {}
}
