package frc.robot.utils.emu

/**
 * Enum class representing the state of the pivot for algae intaking.
 *
 * @property pos The position of the pivot in rotations.
 */
enum class AlgaePivotState(
    @JvmField val pos: Double,
    @JvmField val intakeSpeed: Double,
) {
    /** Represents the pivot when it is ready to intake */
    DOWN(3.5, 4.0),

    /** Represents the pivot when it is up and stowed (most of the time) */
    UP(0.0, 0.0)
}
