package frc.robot.commands;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.emu.AlgaePivotState;
import frc.robot.utils.emu.CoralState;
import frc.robot.utils.emu.ElevatorState;

public class ToggleIntakeAlgae extends Command {
  private final Algae algae = Algae.getInstance();
  private final Coral coral = Coral.getInstance();

  public ToggleIntakeAlgae() {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements();
  }

  @Override
  public void initialize() {
    switch (algaeCounter) {
      case 0:
        algaePivotState = AlgaePivotState.DOWN;
        coralState = CoralState.ALGAE_INTAKE;
        break;
      case 1:
        algaePivotState = AlgaePivotState.HOLD;
        coralState = CoralState.ALGAE_HOLD;
        Elevator.getInstance().setState(ElevatorState.ALGAE_HOLD);
        break;
      case 2:
        algaePivotState = AlgaePivotState.RELEASE;
        coralState = CoralState.ALGAE_RELEASE;
        break;
      case 3:
        algaePivotState = AlgaePivotState.UP;
        coralState = CoralState.CORAL_INTAKE;
        algaeIntaking = false;
        break;
    }
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    algaeCounter++;
  }
}
