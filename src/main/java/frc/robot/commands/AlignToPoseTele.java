package frc.robot.commands;

import static frc.robot.commands.Kommand.moveToClosestCoralScore;
import static frc.robot.commands.Kommand.moveToClosestCoralScoreNotL4;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;
import static frc.robot.utils.RobotParameters.FieldParameters.RobotPoses.addCoralPosList;
import static frc.robot.utils.RobotParameters.LiveRobotValues.visionDead;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;
import static frc.robot.utils.pingu.LogPingu.log;
import static frc.robot.utils.pingu.LogPingu.logs;
import static frc.robot.utils.pingu.PathPingu.clearCoralScoringPositions;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.emu.Direction;
import frc.robot.utils.emu.ElevatorState;

public class AlignToPoseTele extends Command {
  private ProfiledPIDController rotationalController;
  private ProfiledPIDController yController;
  private ProfiledPIDController xController;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private Timer timer;
  private final Swerve swerve;
  private final Direction offsetSide;

  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignToPoseTele(Direction offsetSide) {
    swerve = Swerve.getInstance();
    this.offsetSide = offsetSide;
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    // Update the list of coral scoring positions to the correct side (hopefully)
    currentPose = swerve.getPose2Dfrom3D();

    timer = new Timer();
    addRequirements(swerve);

    clearCoralScoringPositions();
    addCoralPosList();

    if (elevatorToBeSetState == ElevatorState.L4) {
      targetPose = moveToClosestCoralScore(offsetSide, Swerve.getInstance().getPose2Dfrom3D());
    } else {
      targetPose = moveToClosestCoralScoreNotL4(offsetSide, Swerve.getInstance().getPose2Dfrom3D());
    }

    xController = X_PINGU.getProfiledPIDController();
    xController.setTolerance(0.015);
    xController.setConstraints(PROFILE_CONSTRAINTS);
    xController.setGoal(targetPose.getX());
    xController.reset(currentPose.getX());

    yController = Y_PINGU.getProfiledPIDController();
    yController.setTolerance(0.015);
    yController.setConstraints(PROFILE_CONSTRAINTS);
    yController.setGoal(targetPose.getY());
    yController.reset(currentPose.getY());

    rotationalController = ROTATIONAL_PINGU.getProfiledPIDController();
    rotationalController.setTolerance(2.0);
    rotationalController.setConstraints(new TrapezoidProfile.Constraints(5, 5));
    rotationalController.setGoal(targetPose.getRotation().getDegrees());
    rotationalController.reset(currentPose.getRotation().getDegrees());
    rotationalController.enableContinuousInput(-180, 180);
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {
    currentPose = swerve.getPose2Dfrom3D();
    swerve.setDriveSpeeds(
        xController.calculate(currentPose.getX()),
        yController.calculate(currentPose.getY()),
        rotationalController.calculate(currentPose.getRotation().getDegrees()),
        true);
    logs(
        () -> {
          log("AlignToPose/Current Pose", currentPose);
          log("AlignToPose/Target Pose", targetPose);
          log("AlignToPose/Rotational Error", rotationalController.getPositionError());
          log("AlignToPose/Y Error", yController.getPositionError());
          log("AlignToPose/X Error ", xController.getPositionError());
          log("AlignToPose/X Set ", xController.getSetpoint().position);
          log("AlignToPose/X Goal ", xController.getGoal().position);
          log("AlignToPose/Rotational Controller Setpoint", rotationalController.atSetpoint());
          log("AlignToPose/Y Controller Setpoint", yController.atSetpoint());
          log("AlignToPose/X Controller Setpoint ", xController.atSetpoint());
          log(
              "AlignToPose/X Set Speed ",
              xController.calculate(currentPose.getX(), targetPose.getX()));
          log("AlignToPose/Y Set Speed ", yController.calculate(currentPose.getY()));
          log(
              "AlignToPose/Rot Set Speed ",
              rotationalController.calculate(currentPose.getRotation().getDegrees()));
          log("AlignToPose/ X Set Pos", currentPose.getX());
          log("AlignToPose/ Y Set Pos", currentPose.getY());
          log("AlignToPose/ X Target Pos", targetPose.getX());
          log("AlignToPose/ Y Target Pos", targetPose.getY());
        });
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
    if ((rotationalController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint())
        || visionDead) {
      timer.start();
    } else {
      timer.reset();
    }
    return timer.hasElapsed(0.15);
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
