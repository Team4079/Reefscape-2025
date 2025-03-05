package frc.robot.commands.sequencing

import frc.robot.commands.AlignToPoseAuto
import frc.robot.commands.AlignToPoseTele
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
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.DEFAULT

object Sequences {
    @JvmStatic
    fun variableScore(state: ElevatorState) =
        sequential {
            +moveElevatorState(state)
            +setCoralState(CORAL_RELEASE)
            +waitFor(0.3)
            +setElevatorState(DEFAULT)
            +hasPieceFalse()
        }

    @JvmStatic
    fun resetScore() =
        sequential {
            +setCoralState(CORAL_RELEASE)
            +setElevatorState(DEFAULT)
            +cancel()
            +hasPieceFalse()
        }

    @JvmStatic
    fun fullScoreAuto(
        offsetSide: Direction,
        state: ElevatorState,
    ) = sequential {
        +parallel {
//            +moveElevatorState(state)
            +AlignToPoseAuto(offsetSide).withTimeout(0.7157)
        }
        +coralScoring()
        +setCoralState(CORAL_RELEASE)
        +waitFor(0.3)
        // TODO MOVE BACK (prob with on the fly move back 0.5 meter path) jayden u should do this
        // cuz shawn's lazy
        +setElevatorState(DEFAULT)
        +coralScoreFalse()
        +hasPieceFalse()
    }

    @JvmStatic
    fun fullScore(offsetSide: Direction) =
        sequential {
            +parallel {
                +moveElevatorState(elevatorToBeSetState)
                +AlignToPoseTele(offsetSide).withTimeout(1.5)
            }
            +waitFor(0.1)
            +coralScoring()
            +setCoralState(CORAL_RELEASE)
            +waitFor(0.1)
            // TODO: MOVE BACK (prob with on the fly move back 0.5 meter path) jayden u should do this
            // cuz shawn's lazy
            +setElevatorState(DEFAULT)
            +coralScoreFalse()
            +hasPieceFalse()
        }

    @JvmStatic
    fun scoreCoralAuto(offsetSide: Direction) =
        sequential {
            +AlignToPoseAuto(offsetSide).withTimeout(0.9)
            +waitFor(0.1)
            +coralScoring()
            +setCoralState(CORAL_RELEASE)
            +waitFor(1.0)
            // TODO MOVE BACK (prob with on the fly move back 0.5 meter path) jayden u should do this
            // cus im lazy
            +setElevatorState(DEFAULT)
            +coralScoreFalse()
            +hasPieceFalse()
        }
}
