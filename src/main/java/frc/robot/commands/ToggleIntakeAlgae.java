package frc.robot.commands;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.utils.AlgaeStates;
import frc.robot.utils.CoralStates;

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
    if (algaeCounter == 0) {
      algaeIntaking = true;
      algaeState = AlgaeStates.DOWN;
      coralState = CoralStates.ALGAE_INTAKE;
    } else if (algaeCounter == 1) {
      algaeState = AlgaeStates.HOLD;
      coralState = CoralStates.ALGAE_HOLD;
    } else if (algaeCounter == 2) {
      algaeState = AlgaeStates.RELEASE;
      coralState = CoralStates.ALGAE_RELEASE;
    } else if (algaeCounter == 3) {
      algaeState = AlgaeStates.UP;
      coralState = CoralStates.CORAL_INTAKE;
      algaeIntaking = false;
      algaeCounter = -1;
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
