package frc.robot.commands.sequencing;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.ElevatorState.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.utils.*;

/**
 * This command group is used to automatically score the coral manipulator. It aligns the swerve
 * drive, moves the elevator to the correct level, starts the coral manipulator, waits for a second,
 * stops the coral manipulator, sets the elevator to the first level, and moves the elevator to the
 * first level.
 */
public class AutomaticScore extends SequentialCommandGroup {
  public AutomaticScore(Direction offsetSide, ElevatorState state) {
    addCommands(
        //        align(offsetSide),
        moveToElevatorState(state),
        setCoralState(CoralState.CORAL_RELEASE),
        waitCmd(0.3),
        setElevatorState(DEFAULT),
        coralIntaking());
  }
}
