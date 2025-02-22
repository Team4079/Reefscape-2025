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
    L1(10.0),

    /** Represents the second level of the elevator.  */
    L2(15.0),

    /** Represents the third level of the elevator.  */
    L3(34.0),

    /** Represents the fourth level of the elevator.  */
    L4(62.0),

    ALGAE(25.0),

    ALGAE_HOLD(5.0),
}
