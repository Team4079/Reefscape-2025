package frc.robot.commands;

import static frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.SwerveParameters;

public class AlignSwerve extends Command {
  private double yaw;
  private double y;
  private double dist;
  private PIDController rotationalController;
  private PIDController yController;
  private PIDController disController;
  private double
      offset; // double offset is the left/right offset from the april tag to make it properly align

  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignSwerve(Direction offsetSide) {
    switch (offsetSide) {
      case LEFT:
        this.offset = SwerveParameters.AUTO_ALIGN_SWERVE_LEFT;
        break;
      case RIGHT:
        this.offset = SwerveParameters.AUTO_ALIGN_SWERVE_RIGHT;
        break;
      case CENTER:
        this.offset = 0;
        break;
    }

    addRequirements(Swerve.getInstance());
  }

  /**
   * Creates a new AlignSwerve.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   * @param offsetAmount The amount to offset the alignment by.
   */
  public AlignSwerve(Direction offsetSide, double offsetAmount) {
    // TODO: Placeholder for the offset amount, figure out the correct value
    switch (offsetSide) {
      case LEFT:
        this.offset = -offsetAmount;
        break;
      case RIGHT:
        this.offset = offsetAmount;
        break;
      case CENTER:
        this.offset = 0;
        break;
    }

    addRequirements(Swerve.getInstance());
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    yaw = PhotonVision.getInstance().getYaw();
    y = PhotonVision.getInstance().getY();
    dist = PhotonVision.getInstance().getDist();

    rotationalController =
        new PIDController(ROTATIONAL_PID.getP(), ROTATIONAL_PID.getI(), ROTATIONAL_PID.getD());
    // with the L4 branches
    final double tolerance = 0.4;
    rotationalController.setTolerance(tolerance);
    rotationalController.setSetpoint(0);

    yController = new PIDController(Y_PID.getP(), Y_PID.getI(), Y_PID.getD());
    yController.setTolerance(1.5);
    yController.setSetpoint(0);

    disController = new PIDController(DIST_PID.getP(), DIST_PID.getI(), DIST_PID.getD());
    disController.setTolerance(1.5);
    disController.setSetpoint(0);
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    yaw = PhotonVision.getInstance().getYaw();
    y = PhotonVision.getInstance().getY();
    dist = PhotonVision.getInstance().getDist();

    Swerve.getInstance()
        .setDriveSpeeds(
            disController.calculate(dist),
            yController.calculate(y) + offset,
            rotationalController.calculate(yaw),
            false);
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
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
