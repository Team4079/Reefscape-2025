package frc.robot.commands;

import static frc.robot.utils.Dash.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import frc.robot.utils.controller.*;
import kotlin.*;

/** Command to control the robot's swerve drive using a Logitech gaming pad. */
public class PadDrive extends Command {
  private final GamingController pad;
  private final boolean isFieldOriented;

  /**
   * Constructs a new PadDrive command.
   *
   * @param pad The Logitech gaming pad used to control the robot.
   * @param isFieldOriented Whether the drive is field-oriented.
   */
  public PadDrive(GamingController pad, boolean isFieldOriented) {
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;
    addRequirements(Swerve.getInstance());
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. This method retrieves the
   * current position from the gaming pad, calculates the rotation, logs the joystick values, and
   * sets the drive speeds for the swerve subsystem.
   */
  @Override
  public void execute() {
    Pair<Double, Double> position = positionSet(pad);

    double rotation =
        (Math.abs(pad.getRightAnalogXAxis()) >= 0.1)
            ? -pad.getRightAnalogXAxis() * RobotParameters.MotorParameters.MAX_ANGULAR_SPEED
            : 0.0;

    log("X Joystick", position.getFirst());
    log("Y Joystick", position.getSecond());
    log("Rotation", rotation);

    Swerve.getInstance()
        .setDriveSpeeds(position.getSecond(), position.getFirst(), rotation * 0.5, isFieldOriented);
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
  public static Pair<Double, Double> positionSet(GamingController pad) {
    double x = -pad.getLeftAnalogXAxis() * MAX_SPEED;
    if (Math.abs(x) < X_DEADZONE) x = 0.0;

    double y = -pad.getLeftAnalogYAxis() * MAX_SPEED;
    if (Math.abs(y) < Y_DEADZONE) y = 0.0;

    return new Pair<>(x, y);
  }
}
