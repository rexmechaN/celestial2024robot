package com.teamcelestial.network

import edu.wpi.first.networktables.GenericSubscriber
import edu.wpi.first.networktables.NetworkTableInstance

class NetworkValue<T>(
    valueId: String,
    networkValueType: NetworkValueType,
    initialValue: T?
) {
    private val networkTableInstance = NetworkTableInstance.getDefault()
    private val topic = networkTableInstance.getTopic("/datatable/$valueId")
    private val subscription: GenericSubscriber = topic.genericSubscribe(networkValueType.typeName)
    private val publisher = topic.genericPublish(networkValueType.typeName)

    init {
        if(initialValue != null) publisher.setValue(initialValue)
    }

    @Suppress("UNCHECKED_CAST")
    val value: T
        get() = subscription.get().value as T

    fun close() {
        subscription.close()
        publisher.close()
    }
}