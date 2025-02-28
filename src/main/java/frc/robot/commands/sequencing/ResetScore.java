package frc.robot.commands.sequencing;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.emu.ElevatorState.DEFAULT;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.emu.CoralState;

/** */
public class ResetScore extends SequentialCommandGroup {
  public ResetScore() {
    addCommands(
        setCoralState(CoralState.CORAL_RELEASE),
        setElevatorState(DEFAULT),
        cancelCmd(),
        hasPieceFalse());
  }
}
