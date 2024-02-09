package com.teamcelestial.network

import edu.wpi.first.networktables.GenericSubscriber
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import java.util.*
import java.util.concurrent.atomic.AtomicReference

class NetworkValue<T>(
    valueId: String,
    networkValueType: NetworkValueType,
    initialValue: T?
) {
    private var listener: ((T) -> Unit)? = null
    private val networkTableInstance = NetworkTableInstance.getDefault()
    private val topic = networkTableInstance.getTopic("/datatable/$valueId")
    private val subscription: GenericSubscriber = topic.genericSubscribe(networkValueType.typeName)
    private val publisher = topic.genericPublish(networkValueType.typeName)

    private val valReference = AtomicReference<T>()

    init {
        if(initialValue != null) publisher.setValue(initialValue)
        networkTableInstance.addListener(subscription, EnumSet.of(NetworkTableEvent.Kind.kValueAll)) {
            @Suppress("UNCHECKED_CAST")
            (it.valueData.value.value as T).let { value ->
                valReference.set(value)
                listener?.invoke(value)
            }
        }
    }

    val value: T
        get() = valReference.get()

    fun setListener(listener: ((T) -> Unit)?) {
        this.listener = listener
    }

    fun close() {
        subscription.close()
        publisher.close()
    }
}