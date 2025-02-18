package frc.robot.utils.emu

import frc.robot.subsystems.Coral.getInstance

/**
 * Enum class representing the states of the coral manipulator.
 *
 * @property block The function associated with the coral state
 */
enum class CoralState(
    @JvmField val block: Runnable,
) {
    /** Represents the state when the coral manipulator is intaking a coral piece. */
    CORAL_INTAKE(Runnable { getInstance().startCoralIntake() }),

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD(Runnable { getInstance().stopMotors() }),

    /** Represents the state when the coral manipulator is slowing down to hold a coral piece. */
    CORAL_SLOW(Runnable { getInstance().slowCoralIntake() }),

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_RELEASE(Runnable { getInstance().scoreCoral() }),

    /** Represents the state when the coral manipulator is intaking algae. */
    ALGAE_INTAKE(Runnable { getInstance().algaeIntake() }),

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD(Runnable { getInstance().slowAlgaeScoreMotors() }),

    /** Represents the state when the coral manipulator is releasing algae. */
    ALGAE_RELEASE(Runnable { getInstance().ejectAlgae() }),
}
