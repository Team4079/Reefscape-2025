package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Dash;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.RobotParameters;
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds;

/** Command to control the robot's swerve drive using a Logitech gaming pad. */
public class PadDrive extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final LogitechGamingPad pad;
  private final boolean isFieldOriented;

  /**
   * Constructs a new PadDrive command.
   *
   * @param swerveSubsystem The swerve subsystem used by this command.
   * @param pad The Logitech gaming pad used to control the robot.
   * @param isFieldOriented Whether the drive is field-oriented.
   */
  public PadDrive(SwerveSubsystem swerveSubsystem, LogitechGamingPad pad, boolean isFieldOriented) {
    this.swerveSubsystem = swerveSubsystem;
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;
    addRequirements(this.swerveSubsystem);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. This method retrieves the
   * current position from the gaming pad, calculates the rotation, logs the joystick values, and
   * sets the drive speeds for the swerve subsystem.
   */
  @Override
  public void execute() {
    Coordinate position = positionSet(pad);

    double rotation =
        -pad.getRightAnalogXAxis() * RobotParameters.MotorParameters.MAX_ANGULAR_SPEED;
    if (Math.abs(pad.getRightAnalogXAxis()) < 0.2) {
      rotation = 0.0;
    }

    Dash.dash(
        Dash.pairOf("X Joystick", position.x()),
        Dash.pairOf("Y Joystick", position.y()),
        Dash.pairOf("Rotation", rotation));

    swerveSubsystem.setDriveSpeeds(position.y(), position.x(), rotation * 0.8, isFieldOriented);
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

  /** Record representing a coordinate with x and y values. */
  public record Coordinate(double x, double y) {}

  /**
   * Sets the position based on the input from the Logitech gaming pad.
   *
   * @param pad The Logitech gaming pad.
   * @return The coordinate representing the position.
   */
  public static Coordinate positionSet(LogitechGamingPad pad) {
    double x = -pad.getLeftAnalogXAxis() * RobotParameters.MotorParameters.MAX_SPEED;
    if (Math.abs(x) < Thresholds.X_DEADZONE) {
      x = 0.0;
    }

    double y = -pad.getLeftAnalogYAxis() * RobotParameters.MotorParameters.MAX_SPEED;
    if (Math.abs(y) < Thresholds.Y_DEADZONE) {
      y = 0.0;
    }

    return new Coordinate(x, y);
  }
}
