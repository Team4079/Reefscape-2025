package frc.robot.utils.emu

/**
 * Enum representing the different states of the algae intake process.
 */
enum class AlgaeCounter {
    INTAKE, // State when algae is being intaken
    DEFAULT, // Default state

    ;

    /**
     * Returns the next state in the enum sequence.
     * If the current state is the last one, it wraps around to the first state.
     *
     * @return The next AlgaeState.
     */
    val next: AlgaeCounter
        get() = entries[(ordinal + 1) % entries.toTypedArray().size]
}