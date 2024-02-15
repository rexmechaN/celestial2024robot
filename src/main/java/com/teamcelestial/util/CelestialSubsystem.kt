package com.teamcelestial.util

import edu.wpi.first.wpilibj2.command.SubsystemBase

abstract class CelestialSubsystem: SubsystemBase() {
    abstract fun setPower(power: Double)
}