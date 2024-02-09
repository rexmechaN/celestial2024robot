package com.teamcelestial.network

import edu.wpi.first.networktables.GenericSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableValue

class NetworkValue<T>(
    valueId: String,
    valueTypeString: String,
    initialValue: T?
) {
    private val networkTableInstance = NetworkTableInstance.getDefault()
    private val topic = networkTableInstance.getTopic("/datatable/$valueId")
    private val subscription: GenericSubscriber = topic.genericSubscribe(valueTypeString)
    private val publisher = topic.genericPublish(valueTypeString)

    init {
        if(initialValue != null) publisher.set(initialValue as NetworkTableValue)
    }

    val value: T
        get() = subscription.get().value as T

    fun close() {
        subscription.close()
        publisher.close()
    }
}