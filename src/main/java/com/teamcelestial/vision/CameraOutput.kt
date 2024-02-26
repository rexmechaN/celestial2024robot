package com.teamcelestial.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera

class CameraOutput(cameraName: String?) {
    private val camera = PhotonCamera(cameraName)
    val bestTarget: Transform3d?
        get() = camera.latestResult.bestTarget?.bestCameraToTarget

    val allTargets: List<Transform3d>
        get() = camera.latestResult.getTargets().map { it.bestCameraToTarget }
}
