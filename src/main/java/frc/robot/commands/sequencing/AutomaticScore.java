package frc.robot.commands.sequencing;

import static frc.robot.utils.ElevatorState.*;
import static frc.robot.utils.Kommand.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AlignSwerve;
import frc.robot.utils.*;

public class AutomaticScore extends SequentialCommandGroup {
  public AutomaticScore(Direction offsetSide) {
    addCommands(
        new AlignSwerve(offsetSide),
        moveElevatorToLevel(),
        startCoralManipulator(),
        waitCmd(1),
        stopCoralManipulator(),
        setElevatorState(L1),
        moveElevatorToLevel());
  }
}
