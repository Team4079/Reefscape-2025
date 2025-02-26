package frc.robot.commands.sequencing;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.*;
import static frc.robot.utils.emu.ElevatorState.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AlignToPose;
import frc.robot.utils.emu.*;

/**
 * This command group is used to automatically score the coral manipulator. It aligns the swerve
 * drive, moves the elevator to the correct level, starts the coral manipulator, waits for a second,
 * stops the coral manipulator, sets the elevator to the first level, and moves the elevator to the
 * first level.
 */
public class AutomaticScore extends SequentialCommandGroup {
  public AutomaticScore(Direction offsetSide, ElevatorState state, XboxController pad) {
    addCommands(
        new ParallelCommandGroup(
            //          moveElevatorState(elevatorToBeSetState),
            new AlignToPose(offsetSide, pad).withTimeout(2.0)),
        new WaitCommand(0.5),
        setCoralState(CoralState.CORAL_RELEASE),
        waitCmd(0.3),
        // TODO MOVE BACK (prob with on the fly move back 0.5 meter path) jayden u should do this
        // cus im lazy
        //        setElevatorState(DEFAULT),
        coralIntaking());
  }
}
