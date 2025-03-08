package frc.robot.utils.emu

/**
 * The ElevatorState enum represents the different levels that the elevator can be in within the
 * robot's system.
 *
 * @property pos The position of the elevator in rotations.
 */
enum class ElevatorState(
    @JvmField var pos: Double,
) {
    /** Represents the default state of the elevator  */
    DEFAULT(0.1),

    /** Represents the first level of the elevator (aka the trough).  */
    L1(8.0),

    /** Represents the second level of the elevator.  */
    L2(18.0),

    /** Represents the third level of the elevator.  */
    L3(37.0),

    /** Represents the fourth level of the elevator.  */
    L4(64.5),

    ALGAE_LOW(40.0),

    ALGAE_HIGH(58.01)
}