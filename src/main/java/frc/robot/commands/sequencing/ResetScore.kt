package frc.robot.commands.sequencing

import frc.robot.commands.Kommand.cancel
import frc.robot.commands.Kommand.coralIntaking
import frc.robot.commands.Kommand.sequential
import frc.robot.commands.Kommand.setCoralState
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.utils.emu.CoralState.CORAL_RELEASE
import frc.robot.utils.emu.ElevatorState.DEFAULT

/**
 * Creates a sequential command group to reset the scoring mechanism.
 *
 * @return A SequentialCommandGroup that performs the following actions:
 * - Sets the coral state to release.
 * - Sets the elevator state to default.
 * - Cancels any ongoing commands.
 * - Initiates the coral intake process.
 */
fun resetScore() =
    sequential {
        +setCoralState(CORAL_RELEASE) // Set the coral state to release
        +setElevatorState(DEFAULT) // Set the elevator state to default
        +cancel() // Cancel any ongoing commands
        +coralIntaking() // Start the coral intake process
    }
