package com.teamcelestial.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera

class CameraOutput(cameraNickname: String?) {
    private val camera = PhotonCamera(cameraNickname)
    val bestTarget: Transform3d?
        get() = camera.latestResult.bestTarget?.bestCameraToTarget

    val allTargets: List<Transform3d>
        get() = camera.latestResult.getTargets().map { it.bestCameraToTarget }
}
