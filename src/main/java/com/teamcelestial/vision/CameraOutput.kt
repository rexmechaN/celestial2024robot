package com.teamcelestial.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget

class CameraOutput(cameraName: String?) {
    private val camera = PhotonCamera(cameraName)
    val bestTarget: PhotonTrackedTarget?
        get() = camera.latestResult?.bestTarget
}
