package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToPose;
import frc.robot.utils.emu.CoralState;
import frc.robot.utils.emu.Direction;
import frc.robot.utils.emu.ElevatorState;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;
import static frc.robot.utils.emu.ElevatorState.DEFAULT;

/**
 * This command group is used to automatically score the coral manipulator. It aligns the swerve
 * drive, moves the elevator to the correct level, starts the coral manipulator, waits for a second,
 * stops the coral manipulator, sets the elevator to the first level, and moves the elevator to the
 * first level.
 */
public class ResetScore extends SequentialCommandGroup {
  public ResetScore() {
    addCommands(
        setCoralState(CoralState.CORAL_RELEASE),
        setElevatorState(DEFAULT),
        coralIntaking());
  }
}
