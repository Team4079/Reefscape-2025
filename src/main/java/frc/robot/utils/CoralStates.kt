package frc.robot.utils

/**
 * Enum class representing the states of the coral manipulator.
 */
enum class CoralStates {
    /** Represents the state when the coral manipulator is intaking a coral piece. */
    CORAL_INTAKE,

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD,

    /** Represents the state when the coral manipulator is slowing down to hold a coral piece. */
    CORAL_SLOW,

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_RELEASE,

    /** Represents the state when the coral manipulator is intaking algae. */
    ALGAE_INTAKE,

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD,

    /** Represents the state when the coral manipulator is releasing algae. */
    ALGAE_RELEASE,
}
