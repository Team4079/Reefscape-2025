package frc.robot.commands.sequencing

import frc.robot.commands.AlignToPose
import frc.robot.commands.Kommand.cancel
import frc.robot.commands.Kommand.coralIntaking
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.parallel
import frc.robot.commands.Kommand.sequential
import frc.robot.commands.Kommand.setCoralState
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.Kommand.waitFor
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.emu.CoralState.CORAL_RELEASE
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.DEFAULT

object Sequences {
    /**
     * Creates a sequential command group to score a variable.
     *
     * @param state The target state of the elevator.
     * @return A SequentialCommandGroup that performs the following actions:
     * - Moves the elevator to the specified state.
     * - Sets the coral state to release.
     * - Waits for 0.3 seconds.
     * - Resets the elevator state to default.
     * - Initiates the coral intake process.
     */
    @JvmStatic
    fun variableScore(state: ElevatorState) =
        sequential {
            +moveElevatorState(state) // Move the elevator to the specified state
            +setCoralState(CORAL_RELEASE) // Set the coral state to release
            +waitFor(0.3) // Wait for 0.3 seconds
            +setElevatorState(DEFAULT) // Reset the elevator state to default
            +coralIntaking() // Start the coral intake process
        }

    /**
     * Creates a sequential command group to reset the scoring mechanism.
     *
     * @return A SequentialCommandGroup that performs the following actions:
     * - Sets the coral state to release.
     * - Sets the elevator state to default.
     * - Cancels any ongoing commands.
     * - Initiates the coral intake process.
     */
    @JvmStatic
    fun resetScore() =
        sequential {
            +setCoralState(CORAL_RELEASE) // Set the coral state to release
            +setElevatorState(DEFAULT) // Set the elevator state to default
            +cancel() // Cancel any ongoing commands
            +coralIntaking() // Start the coral intake process
        }

    /**
     * Creates a sequential command group to perform a full scoring automation.
     *
     * @param offsetSide The direction to offset the swerve drive.
     * @param state The target state of the elevator.
     * @return A SequentialCommandGroup that performs the following actions:
     * - Aligns the swerve drive and moves the elevator to the specified state in parallel.
     * - Waits for 0.5 seconds.
     * - Sets the coral state to release.
     * - Waits for 0.3 seconds.
     * - Resets the elevator state to default.
     * - Initiates the coral intake process.
     */
    @JvmStatic
    fun fullScoreAuto(
        offsetSide: Direction,
        state: ElevatorState,
    ) = sequential {
        +parallel {
            +moveElevatorState(state) // Move the elevator to the specified state
            +AlignToPose(offsetSide) // Align the swerve drive
        }
        +waitFor(0.5) // Wait for 0.5 seconds
        +setCoralState(CORAL_RELEASE) // Set the coral state to release
        +waitFor(0.3) // Wait for 0.3 seconds
        // TODO MOVE BACK (prob with on the fly move back 0.5 meter path) jayden u should do this
        // cuz shawn's lazy
        +setElevatorState(DEFAULT) // Reset the elevator state to default
        +coralIntaking() // Start the coral intake process
    }

    /**
     * Creates a sequential command group to automatically score the coral manipulator.
     *
     * @param offsetSide The direction to offset the swerve drive.
     * @return A SequentialCommandGroup that performs the following actions:
     * - Aligns the swerve drive and moves the elevator to the correct level in parallel.
     * - Waits for 0.5 seconds.
     * - Sets the coral state to release.
     * - Waits for 0.3 seconds.
     * - Sets the elevator state to default.
     * - Initiates the coral intake process.
     */
    @JvmStatic
    fun fullScore(offsetSide: Direction) =
        sequential {
            +parallel {
                +moveElevatorState(elevatorToBeSetState) // Move the elevator to the specified state
                +AlignToPose(offsetSide) // Align the swerve drive
            }
            +waitFor(0.5) // Wait for 0.5 seconds
            +setCoralState(CORAL_RELEASE) // Set the coral state to release
            +waitFor(0.3) // Wait for 0.3 seconds
            // TODO: MOVE BACK (prob with on the fly move back 0.5 meter path) jayden u should do this
            // cuz shawn's lazy
            +setElevatorState(DEFAULT) // Reset the elevator state to default
            +coralIntaking() // Start the coral intake process
        }
}
