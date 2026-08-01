package frc.robot.utils.misc

data class BiSignal<Type, Type2>(
    val listeners: MutableMap<String, (Type, Type2) -> Unit> = mutableMapOf()
) {
    fun add(name: String, function: (Type, Type2) -> Unit) {
        listeners[name] = function
    }

    fun remove(name: String) {
        listeners.remove(name)
    }

    fun update(input: Type, input2: Type2) {
        listeners.forEach { (_, listenerFunction) ->
            listenerFunction(input, input2)
        }
    }
}
