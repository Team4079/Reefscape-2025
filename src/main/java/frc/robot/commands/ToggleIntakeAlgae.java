package frc.robot.commands;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.emu.*;

public class ToggleIntakeAlgae extends Command {
  private final Elevator elevator = Elevator.getInstance();

  public ToggleIntakeAlgae() {
    addRequirements(Elevator.getInstance());
  }

  // TODO Reverse direction of coral intake motor when intaking algae (double check this works)
  @Override
  public void initialize() {
    switch (algaeCounter) {
      case INTAKE:
        if (elevatorToBeSetState == ElevatorState.L2) {
          algaePivotState = AlgaePivotState.DOWN;
          coralState = CoralState.ALGAE_INTAKE;
          elevator.setState(ElevatorState.ALGAE_LOW);
          algaeIntaking = true;
        } else if (elevatorToBeSetState == ElevatorState.L3) {
          algaePivotState = AlgaePivotState.DOWN;
          coralState = CoralState.ALGAE_INTAKE;
          elevator.setState(ElevatorState.ALGAE_HIGH);
          algaeIntaking = true;
        } else {
          algaeCounter = AlgaeCounter.DEFAULT;
        }
        break;
      case HOLD:
        algaePivotState = AlgaePivotState.HOLD;
        coralState = CoralState.ALGAE_HOLD;
        elevator.setState(ElevatorState.ALGAE_HOLD);
        break;
      case RELEASE:
        algaePivotState = AlgaePivotState.RELEASE;
        coralState = CoralState.ALGAE_RELEASE;
        break;
      case DEFAULT:
        algaePivotState = AlgaePivotState.UP;
        coralState = CoralState.CORAL_INTAKE;
        algaeIntaking = false;
        elevator.setState(ElevatorState.DEFAULT);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    algaeCounter = algaeCounter.getNext();
  }
}
