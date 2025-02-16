package frc.robot.commands;

import static frc.robot.utils.RobotParameters.SwerveParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.utils.*;

public class AlignSwerve extends Command {
  private double yaw;
  private double y;
  private double dist;
  private PIDController rotationalController;
  private PIDController yController;
  private PIDController disController;
  private Timer timer;
  private double offset; // double offset is the left/right offset from the april tag to make it properly align

  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignSwerve(Direction offsetSide) {
    switch (offsetSide) {
      case LEFT:
        this.offset = AUTO_ALIGN_SWERVE_LEFT_OFFSET;
        break;
      case RIGHT:
        this.offset = AUTO_ALIGN_SWERVE_RIGHT_OFFSET;
        break;
      case CENTER:
        this.offset = 0;
        break;
      default:
        throw new IllegalStateException("Unexpected value: " + offsetSide);
    }

    addRequirements(Swerve.getInstance());
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    yaw = PhotonVision.getInstance().getYaw();
    y = PhotonVision.getInstance().getY();
    dist = PhotonVision.getInstance().getDist();

    rotationalController = ROTATIONAL_PINGU.getPidController();
    rotationalController.setTolerance(0.4); // with L4 branches
    rotationalController.setSetpoint(0);

    yController = Y_PINGU.getPidController();
    yController.setTolerance(1.5);
    yController.setSetpoint(0);

    disController = DIST_PINGU.getPidController();
    disController.setTolerance(1.5);
    disController.setSetpoint(0);

    timer = new Timer();
    timer.reset();
    timer.start();
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {
    yaw = PhotonVision.getInstance().getYaw();

    y = PhotonVision.getInstance().getY() + offset;

    dist = PhotonVision.getInstance().getDist();

    Swerve.getInstance()
        .setDriveSpeeds(
            disController.calculate(dist),
            yController.calculate(y),
            rotationalController.calculate(yaw),
            false);

    if (PhotonVision.getInstance().hasTag()) {
      timer.reset();
    }
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link InstantCommand InstantCommand} for such an operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0);
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that is called when {@link #isFinished()} returns true -- or when it is interrupted/canceled.
   * This is where you may want to wrap up loose ends, like shutting off a motor that was being used
   * in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    Swerve.getInstance().stop();
  }
}
