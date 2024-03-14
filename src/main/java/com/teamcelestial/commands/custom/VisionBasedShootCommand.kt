package com.teamcelestial.commands.custom

import com.teamcelestial.Robot
import com.teamcelestial.subsystems.Arm
import com.teamcelestial.subsystems.Rotator
import com.teamcelestial.subsystems.ShooterAssembly
import com.teamcelestial.system.shooter.AbsoluteShooterTarget
import com.teamcelestial.system.shooter.RelativeShooterTarget
import com.teamcelestial.vision.CameraOutput
import edu.wpi.first.wpilibj2.command.Command
import org.photonvision.PhotonUtils

class VisionBasedShootCommand(
    private val arm: Arm,
    private val rotator: Rotator,
    private val armTheta: Double
): Command() {
    private val cameraOutput = CameraOutput("celestial")

    override fun execute() {
        if(cameraOutput.bestTarget != null) {
            val x = cameraOutput.bestTarget?.bestCameraToTarget?.x
            val y = cameraOutput.bestTarget?.bestCameraToTarget?.y
            val z = cameraOutput.bestTarget?.bestCameraToTarget?.z
            if(x != null && y != null && z != null) {
                val target = AbsoluteShooterTarget(x, 1.98)
                ShooterAssembly.aimForTarget(target)
            }
        }
    }
}