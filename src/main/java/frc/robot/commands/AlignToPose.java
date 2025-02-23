package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.emu.Direction;
import kotlin.Pair;
import org.photonvision.PhotonCamera;

import static frc.robot.commands.Kommand.moveToClosestCoralScore;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;
import static frc.robot.utils.pingu.LogPingu.log;
import static frc.robot.utils.pingu.LogPingu.logs;

public class AlignToPose extends Command {
  private double yaw;
  private double y;
  private double dist;
  private PhotonVision photonVision;
  private PIDController rotationalController;
  private PIDController yController;
  private PIDController xController;
  private Pose2d targetPose;
  private Pose2d currentPose;
  private Timer timer;
  private double
      offset; // double offset is the left/right offset from the april tag to make it properly align
  PhotonCamera camera;
    private XboxController pad;
    private Swerve swerve;


  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignToPose(Direction offsetSide, XboxController pad) {

    photonVision = PhotonVision.getInstance();
    swerve = Swerve.getInstance();
    this.pad = pad;
    targetPose = moveToClosestCoralScore(offsetSide, Swerve.getInstance().getPose());
    currentPose = swerve.getPose();
    addRequirements(swerve);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    rotationalController = ROTATIONAL_PINGU.getPidController();
    rotationalController.setTolerance(2.0); // with L4 branches
    rotationalController.setSetpoint(targetPose.getRotation().getDegrees());

    yController = Y_PINGU.getPidController();
    yController.setTolerance(1.0);
    yController.setSetpoint(targetPose.getY());

    xController = X_PINGU.getPidController();
    xController.setTolerance(1.0);
    xController.setSetpoint(targetPose.getX());
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {
    // Using PID for x, y and rotation, align it to the target pose

    currentPose = swerve.getPose();
//    rotationalController.calculate(currentPose.getRotation().getDegrees());
//    yController.calculate(currentPose.getY());
//    xController.calculate(currentPose.getX());


    // Swerve drive set speeds is x y then rotation, so we need to set the speeds in the correct order
    swerve.setDriveSpeeds(
        xController.calculate(currentPose.getX()),
        yController.calculate(currentPose.getY()),
        rotationalController.calculate(currentPose.getRotation().getDegrees()),
        false);


    logs(
        () -> {
          log("AlignToPose/Current Pose", currentPose);
          log("AlignToPose/Target Pose", targetPose);
          log("AlignToPose/Rotational Controller", rotationalController.getError());
          log("AlignToPose/Y Controller", yController.getError());
          log("AlignToPose/X Controller ", xController.getError());
          log("AlignToPose/Rotational Controller Setpoint", rotationalController.getSetpoint());
          log("AlignToPose/Y Controller Setpoint", yController.getSetpoint());
          log("AlignToPose/X Controller Setpoint ", xController.getSetpoint());
        });
  }

  /**
   * Sets the position based on the input from the Logitech gaming pad.
   *
   * @param pad The Logitech gaming pad.
   * @return The coordinate representing the position. The first element is the x-coordinate, and
   *     the second element is the y-coordinate.
   */
  public static Pair<Double, Double> positionSet(XboxController pad) {
      return PadDrive.positionSet(pad);
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
      return rotationalController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
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
