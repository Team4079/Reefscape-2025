package frc.robot.commands;

import static frc.robot.commands.sequencing.Sequences.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import kotlin.*;

/** Command to control the robot's swerve drive using a Logitech gaming pad. */
public class PadElevator extends Command {
  private final XboxController aacrn;
  private final XboxController calamityCow;

  /** Constructs a new PadDrive command. */
  public PadElevator(XboxController aacrn, XboxController calamityCow) {
    this.aacrn = aacrn;
    this.calamityCow = calamityCow;
    addRequirements(Elevator.getInstance());
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. This method retrieves the
   * current position from the gaming pad, calculates the rotation, logs the joystick values, and
   * sets the drive speeds for the swerve subsystem.
   */
  @Override
  public void execute() {
    // Code to manually move elevator
    //    Pair<Double, Double> position = positionSet(pad);
    //    Elevator.getInstance().moveElevator(position.getSecond());

    //    if (checkDPad(0)) {
    //      setElevatorState(L4).schedule();
    //    } else if (checkDPad(2)) {
    //      setElevatorState(ALGAE).schedule();
    //    } else if (checkDPad(4)) {
    //      setElevatorState(L2).schedule();
    //    } else if (checkDPad(6)) {
    //      setElevatorState(L1).schedule();
    //    }

    //    if (aacrn.getYButton()) {
    //      RobotParameters.CoralManipulatorParameters.coralState = CoralState.CORAL_INTAKE;
    //    } else {
    //      RobotParameters.CoralManipulatorParameters.coralState = CoralState.CORAL_HOLD;
    //    }

    // THIS IS WHEN WE HAVE TWO CONTROLLERS,
    // JAYDEN WILL CLICK A DPAD AND AUTOSCORE TAKES THIS VARIABLE AND GOES TO THAT HEIGHT
    if (checkDPad(0)) {
      elevatorToBeSetState = L4;
    } else if (checkDPad(2)) {
      elevatorToBeSetState = L3;
    } else if (checkDPad(4)) {
      elevatorToBeSetState = L2;
    } else if (checkDPad(6)) {
      elevatorToBeSetState = L1;
    }
  }

  /**
   * Check the state of the D-pad. The {@code index} is a value [0, 7] that corresponds to the
   * combinations on the D-pad. 0 represents just 'UP' being pressed, 1 is 'UP-RIGHT', 2 is just
   * 'RIGHT', 3 is 'RIGHT-DOWN', and so on.
   *
   * <p>This method can be used to see if a specific button on the D-pad is pressed.
   *
   * @param index The value to correspond to a D-pad combination.
   * @return If the specified combination is pressed.
   */
  public boolean checkDPad(int index) {
    if (0 <= index && index <= 7) return (index * 45) == calamityCow.getPOV(0);
    else return false;
  }

  /**
   * Returns true when the command should end.
   *
   * @return Always returns false, as this command never ends on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Sets the position based on the input from the Logitech gaming pad.
   *
   * @param pad The Logitech gaming pad.
   * @return The coordinate representing the position. The first element is the x-coordinate, and
   *     the second element is the y-coordinate.
   */
  public static Pair<Double, Double> positionSet(XboxController pad) {
    double x = pad.getLeftX();
    if (Math.abs(x) < X_DEADZONE) x = 0.0;

    double y = pad.getLeftY();
    if (Math.abs(y) < Y_DEADZONE) y = 0.0;

    return new Pair<>(x, y);
  }
}
