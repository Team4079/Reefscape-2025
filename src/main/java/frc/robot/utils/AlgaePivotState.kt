package frc.robot.utils

/**
 * Enum class representing the state of the pivot for algae intaking.
 *
 * @property pos The position of the pivot in rotations.
 */

enum class AlgaePivotState(
    @JvmField var pos: Double,
) {
    /** Represents the pivot when it is ready to intake */
    DOWN(1.0),

    /** Represents the pivot when it is up and stowed (most of the time) */
    UP(0.0),
}
