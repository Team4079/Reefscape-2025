package frc.robot.commands.sequencing

import frc.robot.commands.Kommand.align
import frc.robot.commands.Kommand.alignAuto
import frc.robot.commands.Kommand.cancel
import frc.robot.commands.Kommand.coralScoreFalse
import frc.robot.commands.Kommand.coralScoring
import frc.robot.commands.Kommand.hasPieceFalse
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.parallel
import frc.robot.commands.Kommand.sequential
import frc.robot.commands.Kommand.setCoralState
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.Kommand.waitFor
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.emu.CoralState.CORAL_RELEASE
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState.DEFAULT

object Sequences {
    /**
     * Creates a sequence of commands to reset the scoring mechanism.
     *
     * @return A sequential command group.
     */
    @JvmStatic
    fun resetScore() =
        sequential {
            +setCoralState(CORAL_RELEASE)
            +setElevatorState(DEFAULT)
            +cancel()
            +hasPieceFalse()
        }

    /**
     * Creates a full scoring sequence for the current eleveator state in autonomous mode.
     *
     * Moving the elevator state to L4 and default is already accounted for in event markers.
     *
     * @param offsetSide The direction to offset the alignment.
     * @return A sequential command group.
     */
    @JvmStatic
    fun fullScoreAuto(offsetSide: Direction) =
        sequential {
            +align(offsetSide).withTimeout(1.25)
            +coralScoring()
            +setCoralState(CORAL_RELEASE)
            +waitFor(0.5)
            +coralScoreFalse()
            +hasPieceFalse()
        }

    /**
     * Creates a full scoring sequence for teleoperated mode.
     *
     * @param offsetSide The direction to offset the alignment.
     * @return A sequential command group.
     */
    @JvmStatic
    fun fullScore(offsetSide: Direction) =
        sequential {
            +parallel {
                +moveElevatorState(elevatorToBeSetState)
                +align(offsetSide).withTimeout(2.0)
            }
            +waitFor(0.1)
            +coralScoring()
            +setCoralState(CORAL_RELEASE)
            +waitFor(0.5)
            +setElevatorState(DEFAULT)
            +coralScoreFalse()
            +hasPieceFalse()
        }
}
