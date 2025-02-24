package frc.robot.commands;

import static frc.robot.commands.Kommand.setCoralState;
import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;

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
  private final Elevator elevator = Elevator.getInstance();

  public ToggleIntakeAlgae() {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements();
  }

  // TODO Reverse direction of coral intake motor when intaking algae (double check this works)
  @Override
  public void initialize() {
    switch (algaeCounter) {
      case 0:
        if (elevatorToBeSetState == ElevatorState.L2) {
          algaePivotState = AlgaePivotState.DOWN;
          coralState = CoralState.ALGAE_INTAKE;
          elevator.setState(ElevatorState.ALGAE_LOW);
        } else if (elevatorToBeSetState == ElevatorState.L3) {
          algaePivotState = AlgaePivotState.DOWN;
          coralState = CoralState.ALGAE_INTAKE;
          elevator.setState(ElevatorState.ALGAE_HIGH);
        } else {
          algaeCounter = -1;
          return;
        }
        break;
      case 1:
        algaePivotState = AlgaePivotState.HOLD;
        coralState = CoralState.ALGAE_HOLD;
        elevator.setState(ElevatorState.ALGAE_HOLD);
        break;
      case 2:
        algaePivotState = AlgaePivotState.RELEASE;
        coralState = CoralState.ALGAE_RELEASE;
        break;
      case 3:
        algaePivotState = AlgaePivotState.UP;
        coralState = CoralState.CORAL_INTAKE;
        algaeIntaking = false;
        elevator.setState(ElevatorState.DEFAULT);
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
    if (algaeCounter > 3){
      algaeCounter = 0;
    }
  }
}
