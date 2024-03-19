package com.teamcelestial.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget

class CameraOutput(cameraName: String?) {
    private val camera = PhotonCamera(cameraName)
    val bestTarget: PhotonTrackedTarget?
        get() = if(camera.latestResult.hasTargets()) camera.latestResult.bestTarget else null

    val allTargets: List<PhotonTrackedTarget>
        get() = if(camera.latestResult.hasTargets()) camera.latestResult.getTargets() else listOf()
}
