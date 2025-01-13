package frc.robot.commands.sequencing;

import static frc.robot.utils.ElevatorState.*;
import static frc.robot.utils.Kommand.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AlignSwerve;
import frc.robot.utils.*;

/**
 * This command group is used to automatically score the coral manipulator. It aligns the swerve
 * drive, moves the elevator to the correct level, starts the coral manipulator, waits for a second,
 * stops the coral manipulator, sets the elevator to the first level, and moves the elevator to the
 * first level.
 */
public class AutomaticScore extends SequentialCommandGroup {
  public AutomaticScore(Direction offsetSide) {
    addCommands(
        new AlignSwerve(offsetSide),
//        setElevatorState(L4),
        moveElevatorToLevel(),
        startCoralManipulator(),
        waitCmd(1),
        setElevatorState(L1),
        moveElevatorToLevel());
  }
}
