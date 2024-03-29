package com.teamcelestial.system.assembly

enum class ShooterAssemblyState (
    val index: Int,
) {
    idle(0),
    wandering(1),
    arming(2),
    accelerating(3),
    shooting(4),
}
