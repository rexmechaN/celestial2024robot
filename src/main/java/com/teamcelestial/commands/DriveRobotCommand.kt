package com.teamcelestial.commands

import edu.wpi.first.wpilibj2.command.Command
import com.teamcelestial.subsystems.Drivetrain

class DriveRobotCommand(
    private val xSupplier: () -> Double,
    private val ySupplier: () -> Double
) : Command() {

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {}

    override fun execute() {
        val x = xSupplier()
        val y = ySupplier()
        Drivetrain.drive(x, y)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {}
}
