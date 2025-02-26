package frc.robot.commands.sequencing

import frc.robot.commands.Kommand.coralIntaking
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.sequential
import frc.robot.commands.Kommand.setCoralState
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.Kommand.waitFor
import frc.robot.utils.emu.CoralState.CORAL_RELEASE
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.DEFAULT

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
fun variableScore(state: ElevatorState) =
    sequential {
        +moveElevatorState(state) // Move the elevator to the specified state
        +setCoralState(CORAL_RELEASE) // Set the coral state to release
        +waitFor(0.3) // Wait for 0.3 seconds
        +setElevatorState(DEFAULT) // Reset the elevator state to default
        +coralIntaking() // Start the coral intake process
    }
