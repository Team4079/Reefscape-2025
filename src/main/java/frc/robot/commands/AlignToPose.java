package frc.robot.commands;

import static frc.robot.commands.Kommand.moveToClosestCoralScore;
import static frc.robot.commands.Kommand.moveToClosestCoralScoreNotL4;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;
import static frc.robot.utils.RobotParameters.FieldParameters.RobotPoses.addCoralPosList;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;
import static frc.robot.utils.pingu.LogPingu.log;
import static frc.robot.utils.pingu.LogPingu.logs;
import static frc.robot.utils.pingu.PathPingu.clearCoralScoringPositions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.emu.Direction;
import frc.robot.utils.emu.ElevatorState;
import frc.robot.utils.pingu.NetworkPingu;
import kotlin.Pair;
import org.photonvision.PhotonCamera;

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
  private Direction offsetSide;

  private NetworkPingu networkPinguRotation;
  private NetworkPingu networkPinguY;
  private NetworkPingu networkPinguX;

  private TrapezoidProfile trapProfileX;
  private TrapezoidProfile trapProfileY;
  private TrapezoidProfile trapProfileRot;

  private TrapezoidProfile.State trapProfileXGoal;
  private TrapezoidProfile.State trapProfileXCurrent;
  private TrapezoidProfile.State trapSetX;

  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignToPose(Direction offsetSide, XboxController pad) {

    photonVision = PhotonVision.getInstance();
    swerve = Swerve.getInstance();
    this.offsetSide = offsetSide;
    this.pad = pad;
    currentPose = swerve.getPose2Dfrom3D();
    trapProfileX = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.5, 1.0));
    trapProfileY = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.5, 1.0));
    trapProfileRot = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.5, 1.0));
    trapProfileXCurrent = new TrapezoidProfile.State();
    trapSetX = new TrapezoidProfile.State();

    timer = new Timer();
    addRequirements(swerve);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    // Update the list of coral scoring positions to the correct side (hopefully)
    clearCoralScoringPositions();
    addCoralPosList();

    if (elevatorToBeSetState == ElevatorState.L4) {
      targetPose = moveToClosestCoralScore(offsetSide, Swerve.getInstance().getPose2Dfrom3D());
    } else {
      targetPose = moveToClosestCoralScoreNotL4(offsetSide, Swerve.getInstance().getPose2Dfrom3D());
    }

    trapProfileXCurrent.position = targetPose.getX();

    //    initializeLoggedNetworkPingu();
    //
    //    ROTATIONAL_PINGU.setPID(networkPinguRotation);
    //    Y_PINGU.setPID(networkPinguY);
    //    X_PINGU.setPID(networkPinguX);

    rotationalController = ROTATIONAL_PINGU.getPidController();
    rotationalController.setTolerance(1.3); // with L4 branches
    rotationalController.setSetpoint(targetPose.getRotation().getDegrees());
    rotationalController.enableContinuousInput(-180, 180);

    yController = Y_PINGU.getPidController();
    yController.setTolerance(0.01);
    yController.setSetpoint(targetPose.getY());

//    xController = X_PINGU.getProfiledPIDController();
    xController = X_PINGU.getPidController();
    xController.setTolerance(0.01);
    xController.setSetpoint(targetPose.getX());
//    xController.setConstraints(PROFILE_CONSTANTS);
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {

    // Using PID for x, y and rotation, align it to the target pose

    currentPose = swerve.getPose2Dfrom3D();

    trapProfileXCurrent.position = targetPose.getX();

    trapProfileXGoal = new TrapezoidProfile.State(targetPose.getX(), 0.0);
    trapSetX = trapProfileX.calculate(0.02, trapProfileXCurrent, trapProfileXGoal);
    //    rotationalController.calculate(currentPose.getRotation().getDegrees());
    //    yController.calculate(currentPose.getY());
    //    xController.calculate(currentPose.getX());

    // Swerve drive set speeds is x y then rotation, so we need to set the speeds in the correct
    // order

    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      if (targetPose.getX() < 4.5) {
        swerve.setDriveSpeeds(
          yController.calculate(currentPose.getY(), targetPose.getY()),
          xController.calculate(currentPose.getX(), targetPose.getX()),
          rotationalController.calculate(currentPose.getRotation().getDegrees()),
          false);
      } else {
        swerve.setDriveSpeeds(
          -xController.calculate(currentPose.getX(), targetPose.getX()),
          -yController.calculate(currentPose.getY(), targetPose.getY()),
          rotationalController.calculate(currentPose.getRotation().getDegrees()),
          false);
      }
    } else {
      swerve.setDriveSpeeds(0, 0, 0, false);
    }
    logs(
        () -> {
          log("AlignToPose/Current Pose", currentPose);
          log("AlignToPose/Target Pose", targetPose);
          log("AlignToPose/Rotational Error", rotationalController.getError());
          log("AlignToPose/Y Error", yController.getError());
          log("AlignToPose/X Error ", xController.getPositionError());
          log("AlignToPose/Rotational Controller Setpoint", rotationalController.atSetpoint());
          log("AlignToPose/Y Controller Setpoint", yController.atSetpoint());
          log("AlignToPose/X Controller Setpoint ", xController.atSetpoint());
          log("AlignToPose/X Set Speed ", -xController.calculate(currentPose.getX(), targetPose.getX()));
          log("AlignToPose/Y Set Speed ", yController.calculate(currentPose.getY()));
          log(
              "AlignToPose/Rot Set Speed ",
              rotationalController.calculate(currentPose.getRotation().getDegrees()));
          log("AlignToPose/ X Set Pos", currentPose.getX());
          log("AlignToPose/ Y Set Pos", currentPose.getY());
          log("AlignToPose/ TrapProf set speed x", trapProfileXCurrent.velocity);
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
    //    if (rotationalController.atSetpoint() && yController.atSetpoint() &&
    // xController.atSetpoint()) {
    //      timer.start();
    //    } else {
    //      timer.reset();
    //    }
    //    return timer.hasElapsed(0.5);
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

  //  public void initializeLoggedNetworkPingu() {
  //    networkPinguRotation = new NetworkPingu(new
  // LoggedNetworkNumber("Tuning/AlignToPose/Rotational P", rotationalController.getP()), new
  // LoggedNetworkNumber("Tuning/AlignToPose/Rotational I", rotationalController.getI()), new
  // LoggedNetworkNumber("Tuning/AlignToPose/Rotational D", rotationalController.getD()));
  //    networkPinguY = new NetworkPingu(new LoggedNetworkNumber("Tuning/AlignToPose/Y P",
  // yController.getP()), new LoggedNetworkNumber("Tuning/AlignToPose/Y I", yController.getI()), new
  // LoggedNetworkNumber("Tuning/AlignToPose/Y D", yController.getD()));
  //    networkPinguX = new NetworkPingu(new LoggedNetworkNumber("Tuning/AlignToPose/X P",
  // xController.getP()), new LoggedNetworkNumber("Tuning/AlignToPose/X I", xController.getI()), new
  // LoggedNetworkNumber("Tuning/AlignToPose/X D", xController.getD()));
  //  }
}
